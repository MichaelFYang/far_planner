/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang
 * SPDX-License-Identifier: MIT
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"
#include "far_planner/parameter_validation.h"

namespace {
PointCloudPtr& EnsureCloud(grid_ns::Grid<PointCloudPtr, true>& grid, int index) {
    auto& cloud = grid.GetCell(index);
    if (!cloud) cloud.reset(new PointCloud);
    return cloud;
}
}

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    const int row_num = far_planner::CheckedDimension(
        map_params_.grid_max_length, map_params_.cell_length, "world map width");
    const int col_num = row_num;
    const int level_num = far_planner::CheckedDimension(
        map_params_.grid_max_height, map_params_.cell_height, "world map height", 0, true);
    neighbor_Lnum_ = std::min(row_num, far_planner::CheckedDimension(
        static_cast<double>(map_params_.sensor_range) * 2.0, map_params_.cell_length,
        "map neighborhood width", 1, true));
    neighbor_Hnum_ = 5; 

    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num, col_num, level_num);
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.cell_length, map_params_.cell_length, map_params_.cell_height);
    PointCloudPtr cloud_ptr_tmp;
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr, true>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr, true>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    visited_cell_indices_.clear();

    // init terrain height map
    const int height_dim = far_planner::CheckedDimension(
        (static_cast<double>(map_params_.sensor_range) + map_params_.cell_length) * 2.0,
        FARUtil::robot_dim, "terrain-height grid width", 0, true);
    Eigen::Vector3i height_grid_size(height_dim, height_dim, 1);
    Eigen::Vector3d height_grid_origin(0,0,0);
    Eigen::Vector3d height_grid_resolution(FARUtil::robot_dim, FARUtil::robot_dim, FARUtil::kLeafSize);
    std::vector<float> temp_vec;
    terrain_height_grid_ = std::make_unique<grid_ns::Grid<std::vector<float>>>(
        height_grid_size, temp_vec, height_grid_origin, height_grid_resolution, 3);
    
    const int n_terrain_cell = terrain_height_grid_->GetCellNumber();
    terrain_grid_occupy_list_.resize(n_terrain_cell), terrain_grid_traverse_list_.resize(n_terrain_cell);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);

    INFLATE_N = 1;
    flat_terrain_cloud_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_terrain_clould_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    kdtree_terrain_clould_->setSortedResults(false);
}

void MapHandler::ResetGripMapCloud() {
    for (const int ind : visited_cell_indices_) {
        world_obs_cloud_grid_->EraseCell(ind);
        world_free_cloud_grid_->EraseCell(ind);
    }
    visited_cell_indices_.clear();
    neighbor_obs_indices_.clear();
    neighbor_free_indices_.clear();
    this->ClearTerrainHeightData();
    is_init_ = false;
}

void MapHandler::ClearTerrainHeightData() {
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
    extend_obs_indices_.clear();
    FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
}

void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return;
    const Eigen::Vector3i psub = world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z);
    std::vector<Eigen::Vector3i> ray_subs;
    world_obs_cloud_grid_->RayTraceSubs(robot_cell_sub_, psub, ray_subs);
    const int H = neighbor_Hnum_ / 2;
    for (const auto& sub : ray_subs) {
        for (int k = -H; k <= H; k++) {
            Eigen::Vector3i csub = sub;
            csub.z() += k;
            if (!world_obs_cloud_grid_->InRange(csub)) continue;
            const int ind = world_obs_cloud_grid_->Sub2Ind(csub);
            if (neighbor_obs_indices_.find(ind) == neighbor_obs_indices_.end()) continue;
            world_obs_cloud_grid_->EraseCell(ind);
            const auto free_cloud = world_free_cloud_grid_->GetCellValue(ind);
            if (!free_cloud || free_cloud->empty()) {
                visited_cell_indices_.erase(ind);
            }
        }
    }
}

void MapHandler::GetCloudOfPoint(const Point3D& center, 
                                 const PointCloudPtr& cloudOut,
                                 const CloudType& type,
                                 const bool& is_large) 
{
    cloudOut->clear();
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(center.x, center.y, center.z);
    const int N = is_large ? 1 : 0;
    const int H = neighbor_Hnum_ / 2;
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i csub = sub;
                csub.x() += i, csub.y() += j, csub.z() += k;
                if (!world_obs_cloud_grid_->InRange(csub)) continue;
                if (type == CloudType::FREE_CLOUD) {
                    const auto cell = world_free_cloud_grid_->GetCellValue(csub);
                    if (cell) *cloudOut += *cell;
                } else if (type == CloudType::OBS_CLOUD) {
                    const auto cell = world_obs_cloud_grid_->GetCellValue(csub);
                    if (cell) *cloudOut += *cell;
                } else {
                    if (FARUtil::IsDebug) ROS_ERROR("MH: Assigned cloud type invalid.");
                    return;
                }
            }
        }
    }
}


void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    Point3D map_origin;
    const Eigen::Vector3i dim = world_obs_cloud_grid_->GetSize();
    map_origin.x = ori_robot_pos.x - (map_params_.cell_length * dim.x()) / 2.0f;
    map_origin.y = ori_robot_pos.y - (map_params_.cell_length * dim.y()) / 2.0f;
    map_origin.z = ori_robot_pos.z - (map_params_.cell_height * dim.z()) / 2.0f - FARUtil::vehicle_height; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    world_free_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    is_init_ = true;
    if (FARUtil::IsDebug) ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}

void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);
    terrain_robot_pos_ = odom_pos;
    robot_cell_sub_ = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    // Get neighbor indices
    neighbor_free_indices_.clear(), neighbor_obs_indices_.clear();
    const int N = neighbor_Lnum_ / 2;
    const int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            // additional terrain points -1
            neighbor_sub.z() = robot_cell_sub_.z() - H - 1;
            if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                neighbor_free_indices_.insert(ind);
            }
            for (int k =-H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_obs_indices_.insert(ind), neighbor_free_indices_.insert(ind);
                }
            }
        }
    }
    this->SetTerrainHeightGridOrigin(odom_pos);
}

void MapHandler::SetTerrainHeightGridOrigin(const Point3D& robot_pos) {
    // update terrain height grid center
    const Eigen::Vector3d res = terrain_height_grid_->GetResolution();
    const Eigen::Vector3i dim = terrain_height_grid_->GetSize();
    Eigen::Vector3d grid_origin;
    grid_origin.x() = robot_pos.x - (res.x() * dim.x()) / 2.0f;
    grid_origin.y() = robot_pos.y - (res.y() * dim.y()) / 2.0f;
    grid_origin.z() = 0.0f        - (res.z() * dim.z()) / 2.0f;
    terrain_height_grid_->SetOrigin(grid_origin);
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    obsCloudOut->clear();
    if (!is_init_) return;
    for (const auto& neighbor_ind : neighbor_obs_indices_) {
        const auto cell = world_obs_cloud_grid_->GetCellValue(neighbor_ind);
        if (cell && !cell->empty()) *obsCloudOut += *cell;
    }
}

void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    freeCloudOut->clear();
    if (!is_init_) return;
    for (const auto& neighbor_ind : neighbor_free_indices_) {
        const auto cell = world_free_cloud_grid_->GetCellValue(neighbor_ind);
        if (cell && !cell->empty()) *freeCloudOut += *cell;
    }
}

void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    if (!is_init_ || obsCloudInOut->empty()) return;
    std::vector<int> modified_indices;
    std::unordered_set<int> modified_set;
    PointCloudPtr obs_valid_ptr(new pcl::PointCloud<PCLPoint>());
    for (const auto& point : obsCloudInOut->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        if (neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
            EnsureCloud(*world_obs_cloud_grid_, ind)->push_back(point);
            obs_valid_ptr->push_back(point);
            if (modified_set.insert(ind).second) {
                modified_indices.push_back(ind);
            }
            visited_cell_indices_.insert(ind);
        }
    }
    *obsCloudInOut = *obs_valid_ptr;
    for (const int ind : modified_indices) {
        FARUtil::FilterCloud(world_obs_cloud_grid_->GetCell(ind), FARUtil::kLeafSize);
    }
}

void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    if (!is_init_ || freeCloudIn->empty()) return;
    std::vector<int> modified_indices;
    std::unordered_set<int> modified_set;
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        EnsureCloud(*world_free_cloud_grid_, ind)->push_back(point);
        if (modified_set.insert(ind).second) {
            modified_indices.push_back(ind);
        }
        visited_cell_indices_.insert(ind);
    }
    for (const int ind : modified_indices) {
        FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(ind), FARUtil::kLeafSize);
    }
}

float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {
    is_matched = false;
    const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, 0.0f));
    if (terrain_height_grid_->InRange(sub)) {
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (terrain_grid_traverse_list_[ind] != 0) {
            is_matched = true;
            return terrain_height_grid_->GetCell(ind)[0];
        }
    }
    if (is_search) {
        float matched_dist_squre;
        const float terrain_h = NearestHeightOfPoint(p, matched_dist_squre);
        return terrain_h;
    }
    return p.z; 
}

float MapHandler::NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated) {
    const float p_th = point.z-FARUtil::vehicle_height;
    const Eigen::Vector3i ori_sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, p_th));
    is_associated = false;
    if (world_free_cloud_grid_->InRange(ori_sub)) {
        // downward seach
        bool is_dw_associated = false;
        Eigen::Vector3i dw_near_sub = ori_sub;
        float dw_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(dw_near_sub)) {
            const auto cell = world_free_cloud_grid_->GetCellValue(dw_near_sub);
            if (cell && !cell->empty()) {
                int counter = 0;
                dw_terrain_h = 0.0f;
                for (const auto& pcl_p : cell->points) {
                    dw_terrain_h += pcl_p.z, counter ++;
                }
                dw_terrain_h /= (float)counter;
                is_dw_associated = true;
                break;
            }
            dw_near_sub.z() --;
        }
        // upward search
        bool is_up_associated = false;
        Eigen::Vector3i up_near_sub = ori_sub;
        float up_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(up_near_sub)) {
            const auto cell = world_free_cloud_grid_->GetCellValue(up_near_sub);
            if (cell && !cell->empty()) {
                int counter = 0;
                up_terrain_h = 0.0f;
                for (const auto& pcl_p : cell->points) {
                    up_terrain_h += pcl_p.z, counter ++;
                }
                up_terrain_h /= (float)counter;
                is_up_associated = true;
                break;
            }
            up_near_sub.z() ++;
            
        }
        is_associated = (is_up_associated || is_dw_associated) ? true : false;
        if (is_up_associated && is_dw_associated) { // compare nearest
            if (up_near_sub.z() - ori_sub.z() < ori_sub.z() - dw_near_sub.z()) return up_terrain_h;
            else return dw_terrain_h;
        } else if (is_up_associated) return up_terrain_h;
        else return dw_terrain_h;
    }
    return p_th;
}


bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& point, const bool& is_extend) {
    const float h = point.z - FARUtil::vehicle_height; 
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, h));
    if (!world_obs_cloud_grid_->InRange(sub)) return false;
    const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
    if (is_extend && extend_obs_indices_.find(ind) != extend_obs_indices_.end()) {
        return true;
    }
    if (!is_extend && neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
        return true;
    }
    return false;
}


void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes) {
    if (nodes.empty()) return;
    for (const auto& node_ptr : nodes) {
        if (!node_ptr->is_active || node_ptr->is_boundary || FARUtil::IsFreeNavNode(node_ptr) || FARUtil::IsOutsideGoal(node_ptr) || !FARUtil::IsPointInLocalRange(node_ptr->position, true)) {
            continue;
        } 
        bool is_match = false;
        float terrain_h = TerrainHeightOfPoint(node_ptr->position, is_match, false);
        if (is_match) {
            terrain_h += FARUtil::vehicle_height;
            if (node_ptr->pos_filter_vec.empty()) {
                node_ptr->position.z = terrain_h;
            } else {
                node_ptr->pos_filter_vec.back().z = terrain_h; // assign to position filter
                node_ptr->position.z = FARUtil::AveragePoints(node_ptr->pos_filter_vec).z;
            }
        }
    }
}

void MapHandler::AdjustCTNodeHeight(const CTNodeStack& ctnodes) {
    if (ctnodes.empty()) return;
    const float H_MAX = FARUtil::robot_pos.z + FARUtil::kTolerZ;
    const float H_MIN = FARUtil::robot_pos.z - FARUtil::kTolerZ;
    for (auto& ctnode_ptr : ctnodes) {
        float min_th, max_th;
        const float avg_h = NearestHeightOfRadius(ctnode_ptr->position, FARUtil::kMatchDist, min_th, max_th, ctnode_ptr->is_ground_associate);
        if (ctnode_ptr->is_ground_associate) {
            ctnode_ptr->position.z = min_th + FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN);
        } else {
            ctnode_ptr->position.z = TerrainHeightOfPoint(ctnode_ptr->position, ctnode_ptr->is_ground_associate, true);
            ctnode_ptr->position.z += FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN);
        }
    }
}

void MapHandler::ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs, std::unordered_set<int>& extend_terrain_obs) {
    std::unordered_set<int> neighbor_copy = neighbor_obs;
    neighbor_obs.clear();
    const float R = map_params_.cell_length * 0.7071f; // sqrt(2)/2
    for (const auto& idx : neighbor_copy) {
        const Point3D pos = Point3D(world_obs_cloud_grid_->Ind2Pos(idx)); 
        const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(pos.x, pos.y, 0.0f));
        const int terrain_ind = terrain_height_grid_->Sub2Ind(sub);
        bool inRange = false;
        float minH, maxH;
        const float avgH = NearestHeightOfRadius(pos, R, minH, maxH, inRange);
        if (inRange && pos.z + map_params_.cell_height > minH &&
                       pos.z - map_params_.cell_height < maxH + FARUtil::kTolerZ) // use map_params_.cell_height/2.0 as a tolerance margin
        {
            neighbor_obs.insert(idx);
        }
    }
    extend_terrain_obs.clear(); // assign extended terrain obs indices
    const std::vector<int> inflate_vec{-1, 0};
    for (const int& idx : neighbor_obs) {
        const Eigen::Vector3i csub = world_obs_cloud_grid_->Ind2Sub(idx);
        for (const int& plus : inflate_vec) {
            Eigen::Vector3i sub = csub; 
            sub.z() += plus;
            if (!world_obs_cloud_grid_->InRange(sub)) continue;
            const int plus_idx = world_obs_cloud_grid_->Sub2Ind(sub);
            extend_terrain_obs.insert(plus_idx);
        }
    }
}

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    if (freeCloudIn->empty()) {
        terrainHeightOut->clear();
        this->ClearTerrainHeightData();
        return;
    }
    PointCloudPtr copy_free_ptr(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*freeCloudIn, *copy_free_ptr);
    FARUtil::FilterCloud(copy_free_ptr, terrain_height_grid_->GetResolution());
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    for (const auto& point : copy_free_ptr->points) {
        Eigen::Vector3i csub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, 0.0f));
        std::vector<Eigen::Vector3i> subs;
        this->Expansion2D(csub, subs, INFLATE_N);
        for (const auto& sub : subs) {
            if (!terrain_height_grid_->InRange(sub)) continue;
            const int ind = terrain_height_grid_->Sub2Ind(sub);
            if (terrain_grid_occupy_list_[ind] == 0) {
                terrain_height_grid_->GetCell(ind).resize(1);
                terrain_height_grid_->GetCell(ind)[0] = point.z;
            } else {
                terrain_height_grid_->GetCell(ind).push_back(point.z);
            }
            terrain_grid_occupy_list_[ind] = 1;
        }
    }
    const int N = terrain_grid_occupy_list_.size();
    this->TraversableAnalysis(terrainHeightOut);
    if (terrainHeightOut->empty()) { // set terrain height kdtree
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
    } else {
        this->AssignFlatTerrainCloud(terrainHeightOut, flat_terrain_cloud_);
        kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
    }
    // update surrounding obs cloud grid indices based on terrain
    this->ObsNeighborCloudWithTerrain(neighbor_obs_indices_, extend_obs_indices_);
}

void MapHandler::TraversableAnalysis(const PointCloudPtr& terrainHeightOut) {
    const Eigen::Vector3i robot_sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(terrain_robot_pos_.x,
                                                                                    terrain_robot_pos_.y, 0.0f));
    terrainHeightOut->clear();
    if (!terrain_height_grid_->InRange(robot_sub)) {
        ROS_ERROR("MH: terrain height analysis error: robot position is not in range");
        return;
    }
    const float H_THRED = map_params_.height_voxel_dim;
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
    // Lambda Function
    auto IsTraversableNeighbor = [&] (const int& cur_id, const int& ref_id) {
        if (terrain_grid_occupy_list_[ref_id] == 0) return false;
        const float cur_h = terrain_height_grid_->GetCell(cur_id)[0];
        float ref_h = 0.0f;
        int counter = 0;
        for (const auto& e : terrain_height_grid_->GetCell(ref_id)) {
            if (abs(e - cur_h) > H_THRED) continue;
            ref_h += e, counter ++;
        }
        if (counter > 0) {
            terrain_height_grid_->GetCell(ref_id).resize(1);
            terrain_height_grid_->GetCell(ref_id)[0] = ref_h / (float)counter;
            return true;
        }
        return false;
    };

    auto AddTraversePoint = [&] (const int& idx) {
        Eigen::Vector3d cpos = terrain_height_grid_->Ind2Pos(idx);
        cpos.z() = terrain_height_grid_->GetCell(idx)[0];
        const PCLPoint p = FARUtil::Point3DToPCLPoint(Point3D(cpos));
        terrainHeightOut->push_back(p);
        terrain_grid_traverse_list_[idx] = 1;
    };

    const int robot_idx = terrain_height_grid_->Sub2Ind(robot_sub);
    const std::array<int, 4> dx = {-1, 0, 1, 0};
    const std::array<int, 4> dy = { 0, 1, 0,-1};
    std::deque<int> q;
    bool is_robot_terrain_init = false;
    std::unordered_set<int> visited_set;
    q.push_back(robot_idx), visited_set.insert(robot_idx);
    while (!q.empty()) {
        const int cur_id = q.front();
        q.pop_front();
        if (terrain_grid_occupy_list_[cur_id] != 0) {
            if (!is_robot_terrain_init) {
                float avg_h = 0.0f;
                int counter = 0;
                for (const auto& e : terrain_height_grid_->GetCell(cur_id)) {
                    if (abs(e - terrain_robot_pos_.z + FARUtil::vehicle_height) > H_THRED) continue;
                    avg_h += e, counter ++;
                }
                if (counter > 0) {
                    avg_h /= (float)counter;
                    terrain_height_grid_->GetCell(cur_id).resize(1);
                    terrain_height_grid_->GetCell(cur_id)[0] = avg_h;
                    AddTraversePoint(cur_id);
                    is_robot_terrain_init = true; // init terrain height map current robot height
                    q.clear();
                }
            } else {
                AddTraversePoint(cur_id);
            }
        } else if (is_robot_terrain_init) {
            continue;
        }
        const Eigen::Vector3i csub = terrain_height_grid_->Ind2Sub(cur_id);
        for (int i=0; i<4; i++) {
            Eigen::Vector3i ref_sub = csub;
            ref_sub.x() += dx[i], ref_sub.y() += dy[i];
            if (!terrain_height_grid_->InRange(ref_sub)) continue;
            const int ref_id = terrain_height_grid_->Sub2Ind(ref_sub);
            if (!visited_set.count(ref_id) && (!is_robot_terrain_init || IsTraversableNeighbor(cur_id, ref_id))) {
                q.push_back(ref_id);
                visited_set.insert(ref_id);
            }
        }
    }
}


void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    neighbor_centers.clear();
    if (!is_init_) return;
    for (const auto& ind : neighbor_obs_indices_) {
        if (visited_cell_indices_.count(ind) == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    occupancy_centers.clear();
    if (!is_init_) return;
    for (const int ind : visited_cell_indices_) {
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::vector<int> marked_indices;
    std::unordered_set<int> marked_set;
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        if (marked_set.insert(ind).second) {
            marked_indices.push_back(ind);
        }
    }
    for (const int ind : marked_indices) {
        if (neighbor_obs_indices_.count(ind) != 0 && visited_cell_indices_.count(ind) != 0) {
            const auto cell = world_obs_cloud_grid_->GetCellValue(ind);
            if (cell) {
                FARUtil::RemoveOverlapCloud(cell, obsCloud);
                if (cell->empty()) {
                    world_obs_cloud_grid_->EraseCell(ind);
                    const auto free_cell = world_free_cloud_grid_->GetCellValue(ind);
                    if (!free_cell || free_cell->empty()) visited_cell_indices_.erase(ind);
                }
            }
        }
    }
}

