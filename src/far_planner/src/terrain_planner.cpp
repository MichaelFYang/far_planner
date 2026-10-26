/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang
 * SPDX-License-Identifier: MIT
 * fanyang2@andrew.cmu.edu,   
 */


#include "far_planner/terrain_planner.h"
#include "far_planner/parameter_validation.h"

#include <functional>

/***************************************************************************************/

void TerrainPlanner::Init(const ros::NodeHandle& nh, const TerrainPlannerParams& params) {
    nh_ = nh;
    tp_params_ = params;
    is_grids_init_ = false;
    terrain_map_dirty_ = true;
    terrain_map_msg_ = sensor_msgs::PointCloud2();
    occupied_cells_.clear();
    searched_cells_.clear();
    local_obs_cloud_.reset();
    local_terrain_revision_ = 0;
    needs_projection_ = true;
    last_projected_revision_ = 0;
    row_num_ = far_planner::CheckedDimension(
        (static_cast<double>(FARUtil::kLocalPlanRange) + tp_params_.radius) * 2.0,
        tp_params_.voxel_size, "local terrain grid width");
    col_num_ = row_num_;
    TerrainNodePtr init_terrain_node_ptr = NULL;
    Eigen::Vector3i grid_size(row_num_, col_num_, 1);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(tp_params_.voxel_size, tp_params_.voxel_size, tp_params_.voxel_size);
    terrain_grids_ = std::make_unique<grid_ns::Grid<TerrainNodePtr>>(grid_size, init_terrain_node_ptr, grid_origin, grid_resolution, 3);
    this->AllocateGridNodes(); 
    viz_path_stack_.clear();

    local_path_pub_   = nh_.advertise<Marker>("/local_terrain_path_debug", 5);
    terrain_map_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/local_terrain_map_debug", 5);
}

void TerrainPlanner::UpdateCenterNode(const NavNodePtr& node_ptr) {
    if (node_ptr == NULL) return;
    center_node_prt_ = node_ptr, center_pos_ = node_ptr->position;
    Eigen::Vector3d grid_origin;
    grid_origin.x() = center_pos_.x - (tp_params_.voxel_size * row_num_) / 2.0f;
    grid_origin.y() = center_pos_.y - (tp_params_.voxel_size * col_num_) / 2.0f;
    grid_origin.z() = center_pos_.z - (tp_params_.voxel_size * 1.0f) / 2.0f;
    terrain_grids_->SetOrigin(grid_origin);
    is_grids_init_ = true;
    this->ResetGridsPositions();
    this->ResetGridsOccupancy();
    needs_projection_ = true;
    terrain_map_dirty_ = true;
    if (local_obs_cloud_) {
        this->SetLocalTerrainObsCloud(local_obs_cloud_, local_terrain_revision_);
    }
}

void TerrainPlanner::SetLocalTerrainObsCloud(const PointCloudPtr& obsCloudIn, std::uint64_t revision) {
    local_obs_cloud_ = obsCloudIn;
    local_terrain_revision_ = revision;
    if (!is_grids_init_) return;
    if (!needs_projection_ && revision == last_projected_revision_) {
        this->GridVisualCloud();
        return;
    }
    this->ResetGridsOccupancy();
    const int N_IF = tp_params_.inflate_size;
    for (const auto& point : obsCloudIn->points) {
        Eigen::Vector3i c_sub = terrain_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, center_pos_.z));
        c_sub.z() = 0;
        terrain_grids_->ForEachXYCellInRadius(c_sub, N_IF, [&](int ind) {
            auto& cell = terrain_grids_->GetCell(ind);
            if (!cell->is_occupied) {
                cell->is_occupied = true;
                occupied_cells_.push_back(ind);
            }
        });
    }
    last_projected_revision_ = revision;
    needs_projection_ = false;
    terrain_map_dirty_ = true;
    /* visualize terrain planner map and path */
    this->GridVisualCloud();
}

void TerrainPlanner::GridVisualCloud() {
    if (!is_grids_init_ || terrain_map_pub_.getNumSubscribers() == 0) return;
    if (terrain_map_dirty_) {
        PointCloud temp_cloud;
        const int N = terrain_grids_->GetCellNumber();
        temp_cloud.points.reserve(N);
        for (int ind=0; ind<N; ind++) {
            if (!terrain_grids_->GetCell(ind)->is_occupied) {
                temp_cloud.push_back(this->Ind2PCLPoint(ind));
            }
        }
        pcl::toROSMsg(temp_cloud, terrain_map_msg_);
        terrain_map_msg_.header.frame_id = tp_params_.world_frame;
        terrain_map_dirty_ = false;
    }
    terrain_map_msg_.header.stamp = ros::Time::now();
    terrain_map_pub_.publish(terrain_map_msg_);
}

bool TerrainPlanner::PlanPathFromPToP(const Point3D& from_p, const Point3D& to_p, PointStack& path) {
    path.clear();
    if (!is_grids_init_) return false;
    this->ResetGridsPlanStatus();
    const Eigen::Vector3d start_pos(from_p.x, from_p.y, center_pos_.z);
    const Eigen::Vector3d end_pos(to_p.x, to_p.y, center_pos_.z);
    const Eigen::Vector3i start_sub = terrain_grids_->Pos2Sub(start_pos);
    const Eigen::Vector3i end_sub   = terrain_grids_->Pos2Sub(end_pos);
    if (!terrain_grids_->InRange(start_sub) || !terrain_grids_->InRange(end_sub)) {
        if (FARUtil::IsDebug) ROS_WARN("TP: two interval navigation nodes are not in terrain planning range.");
        return false;
    }
    const TerrainNodePtr start_node_ptr = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(start_sub));
    const TerrainNodePtr end_node_ptr = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(end_sub));
    const Point3D unit_axial = (to_p - from_p).normalize();
    const float ndist = (to_p - from_p).norm();

    // Lambda function
    auto InCylinder = [&](const TerrainNodePtr& tnode_ptr) {
        const Point3D vec = tnode_ptr->position - from_p;
        float proj_scalar = vec * unit_axial;
        if (proj_scalar < - FARUtil::kNavClearDist || proj_scalar > ndist + FARUtil::kNavClearDist) {
            return false;
        }
        const Point3D vec_axial = unit_axial * proj_scalar; 
        if ((vec - vec_axial).norm() > tp_params_.radius) {
            return false;
        }
        return true;
    };

    std::array<int, 8> dx = {1, 1, 0,-1,-1, 0, 1,-1};
    std::array<int, 8> dy = {0, 1, 1, 0, 1,-1,-1,-1};
    start_node_ptr->gscore = 0.0;
    searched_cells_.push_back(start_node_ptr->id);
    using SearchEntry = std::pair<float, TerrainNodePtr>;
    std::priority_queue<SearchEntry, std::vector<SearchEntry>, std::greater<SearchEntry>> open_queue;
    std::unordered_set<int> close_set;
    start_node_ptr->fscore = this->EulerCost(start_node_ptr, end_node_ptr);
    open_queue.push({start_node_ptr->fscore, start_node_ptr});
    while (!open_queue.empty()) {
        const auto entry = open_queue.top();
        open_queue.pop();
        const TerrainNodePtr current = entry.second;
        if (entry.first != current->fscore || close_set.count(current->id)) continue;
        if (current == end_node_ptr) {
            this->ExtractPath(end_node_ptr, path);
            break;
        }
        close_set.insert(current->id);
        const Eigen::Vector3i current_sub = terrain_grids_->Ind2Sub(current->id);
        for (int i=0; i<8; i++) {
            Eigen::Vector3i csub = current_sub;
            csub.x() += dx[i], csub.y() += dy[i], csub.z() = 0;
            if (!terrain_grids_->InRange(csub)) continue;
            if (dx[i] != 0 && dy[i] != 0) {
                Eigen::Vector3i side_sub = current_sub;
                side_sub.x() += dx[i];
                if (terrain_grids_->GetCell(side_sub)->is_occupied) continue;
                side_sub = current_sub;
                side_sub.y() += dy[i];
                if (terrain_grids_->GetCell(side_sub)->is_occupied) continue;
            }
            const TerrainNodePtr neighbor = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(csub));
            if (close_set.count(neighbor->id) || (neighbor->is_occupied && neighbor != end_node_ptr) || !InCylinder(neighbor)) continue;
            const float temp_gscore = current->gscore + this->EulerCost(current, neighbor);
            if (temp_gscore < neighbor->gscore) {
                if (neighbor->gscore == FARUtil::kINF) searched_cells_.push_back(neighbor->id);
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                neighbor->fscore = temp_gscore + this->EulerCost(neighbor, end_node_ptr);
                open_queue.push({neighbor->fscore, neighbor});
            }
        }
    }
    if (!path.empty()) {
        viz_path_stack_.push_back(path);
    }
    return !path.empty();
}


void TerrainPlanner::ExtractPath(const TerrainNodePtr& end_ptr, PointStack& path) {
    path.clear();
    TerrainNodePtr cur_ptr = end_ptr;
    while (true) {
        path.push_back(cur_ptr->position);
        if (cur_ptr->parent != NULL) {
            cur_ptr = cur_ptr->parent;
        } else {
            break;
        }
    }
    std::reverse(path.begin(), path.end());
}

void TerrainPlanner::VisualPaths() {
    Marker terrain_paths_marker;
    terrain_paths_marker.type = Marker::LINE_LIST;
    DPVisualizer::SetMarker(VizColor::ORANGE, "terrain_path", 0.3f, 0.85f, terrain_paths_marker);
    
    auto DrawPath = [&](const PointStack& path) {
        if (path.size() < 2) return;
        geometry_msgs::Point last_p = FARUtil::Point3DToGeoMsgPoint(path[0]);
        for (int i=1; i<path.size(); i++) {
            terrain_paths_marker.points.push_back(last_p);
            last_p = FARUtil::Point3DToGeoMsgPoint(path[i]);
            terrain_paths_marker.points.push_back(last_p);
        }
    };

    for (const auto& tpath : viz_path_stack_) {
        DrawPath(tpath);
    }
    local_path_pub_.publish(terrain_paths_marker);
    viz_path_stack_.clear();
}

