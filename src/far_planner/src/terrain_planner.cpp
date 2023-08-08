/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */


#include "far_planner/terrain_planner.h"

/***************************************************************************************/

void TerrainPlanner::Init(const rclcpp::Node::SharedPtr nh, const TerrainPlannerParams& params) {
    nh_ = nh;
    tp_params_ = params;
    row_num_ = std::ceil((FARUtil::kLocalPlanRange + tp_params_.radius) * 2.0f / tp_params_.voxel_size);
    col_num_ = row_num_;
    TerrainNodePtr init_terrain_node_ptr = NULL;
    Eigen::Vector3i grid_size(row_num_, col_num_, 1);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(tp_params_.voxel_size, tp_params_.voxel_size, tp_params_.voxel_size);
    terrain_grids_ = std::make_unique<grid_ns::Grid<TerrainNodePtr>>(grid_size, init_terrain_node_ptr, grid_origin, grid_resolution, 3);
    this->AllocateGridNodes(); 
    viz_path_stack_.clear();

    local_path_pub_   = nh_->create_publisher<Marker>("/local_terrain_path_debug", 5);
    terrain_map_pub_  = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/local_terrain_map_debug", 5);
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
}

void TerrainPlanner::SetLocalTerrainObsCloud(const PointCloudPtr& obsCloudIn) {
    if (!is_grids_init_ || obsCloudIn->empty()) return;
    const int N_IF = tp_params_.inflate_size;
    for (const auto& point : obsCloudIn->points) {
        Eigen::Vector3i c_sub = terrain_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, center_pos_.z));
        for (int i = -N_IF; i <= N_IF; i++) {
            for (int j = -N_IF; j <= N_IF; j++) {
                Eigen::Vector3i sub;
                sub.x() = c_sub.x() + i, sub.y() = c_sub.y() + j, sub.z() = 0;
                if (terrain_grids_->InRange(sub)) {
                    const int ind = terrain_grids_->Sub2Ind(sub);
                    terrain_grids_->GetCell(ind)->is_occupied = true;
                }
            }
        }
    }
    /* visualize terrain planner map and path */
    this->GridVisualCloud();
}

void TerrainPlanner::GridVisualCloud() {
    if (!is_grids_init_) return;
    PointCloudPtr temp_cloud_ptr(new pcl::PointCloud<PCLPoint>());
    const int N = terrain_grids_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (!terrain_grids_->GetCell(ind)->is_occupied) {
            temp_cloud_ptr->points.push_back(this->Ind2PCLPoint(ind));
        }
    }
    sensor_msgs::msg::PointCloud2 msg_pc;
    pcl::toROSMsg(*temp_cloud_ptr, msg_pc);
    msg_pc.header.frame_id = tp_params_.world_frame;
    msg_pc.header.stamp = nh_->now();
    terrain_map_pub_->publish(msg_pc);
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
        if (FARUtil::IsDebug) RCLCPP_WARN(nh_->get_logger(), "TP: two interval navigation nodes are not in terrain planning range.");
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
    std::unordered_set<int> open_set;
    std::priority_queue<TerrainNodePtr, std::vector<TerrainNodePtr>, TNodeptr_fcomp> open_queue;
    std::unordered_set<int> close_set;
    open_queue.push(start_node_ptr);
    open_set.insert(start_node_ptr->id);
    while (true) {
        if (open_set.empty()) {
            break;
        }
        const TerrainNodePtr current = open_queue.top();
        if (current == end_node_ptr) {
            this->ExtractPath(end_node_ptr, path);
            break;
        }
        open_queue.pop();
        open_set.erase(current->id);
        close_set.insert(current->id);
        for (int i=0; i<8; i++) {
            Eigen::Vector3i csub = terrain_grids_->Ind2Sub(current->id);
            csub.x() += dx[i], csub.y() += dy[i], csub.z() = 0;
            if (!terrain_grids_->InRange(csub)) continue;
            const TerrainNodePtr neighbor = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(csub));
            if (close_set.count(neighbor->id) || (neighbor->is_occupied && neighbor != end_node_ptr) || !InCylinder(neighbor)) continue;
            const float temp_gscore = current->gscore + this->EulerCost(current, neighbor);
            if (temp_gscore < neighbor->gscore) {
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                neighbor->fscore = temp_gscore + this->EulerCost(neighbor, end_node_ptr);
                if (!open_set.count(neighbor->id)) {
                    open_queue.push(neighbor);
                    open_set.insert(neighbor->id);
                } 
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
    DPVisualizer::SetMarker(nh_, VizColor::ORANGE, "terrain_path", 0.3f, 0.85f, terrain_paths_marker);
    
    auto DrawPath = [&](const PointStack& path) {
        if (path.size() < 2) return;
        geometry_msgs::msg::Point last_p = FARUtil::Point3DToGeoMsgPoint(path[0]);
        for (std::size_t i=1; i<path.size(); i++) {
            terrain_paths_marker.points.push_back(last_p);
            last_p = FARUtil::Point3DToGeoMsgPoint(path[i]);
            terrain_paths_marker.points.push_back(last_p);
        }
    };

    for (const auto& tpath : viz_path_stack_) {
        DrawPath(tpath);
    }
    local_path_pub_->publish(terrain_paths_marker);
    viz_path_stack_.clear();
}

