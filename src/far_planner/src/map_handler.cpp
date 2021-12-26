/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    row_num_ = std::ceil(map_params_.grid_max_length / map_params_.ceil_length);
    col_num_ = row_num_;
    level_num_ = std::ceil(map_params_.grid_max_height / map_params_.ceil_height);
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.ceil_length) + 1; 
    neighbor_Hnum_ = 5; 
    if (level_num_ % 2 == 0) level_num_ ++;         // force to odd number, robot will be at center
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++; // force to odd number
    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num_, col_num_, level_num_);
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.ceil_length, map_params_.ceil_length, map_params_.ceil_height);
    PointCloudPtr cloud_ptr_tmp;
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    const int N_CELL  = world_obs_cloud_grid_->GetCellNumber();

    for (int i = 0; i < N_CELL; i++) {
        world_obs_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
        world_free_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
    }

    global_visited_induces_.resize(N_CELL), util_remove_check_list_.resize(N_CELL);
    util_obs_modified_list_.resize(N_CELL), util_free_modified_list_.resize(N_CELL);
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);

    // init terrain height map
    height_dim_ = std::ceil((map_params_.sensor_range * 2.0f + map_params_.ceil_length) / map_params_.height_voxel_dim);
    if (height_dim_ % 2 == 0) height_dim_ ++;
    Eigen::Vector3i height_grid_size(height_dim_, height_dim_, 1);
    Eigen::Vector3d height_grid_origin(0,0,0);
    Eigen::Vector3d height_grid_resolution(map_params_.height_voxel_dim, 
                                           map_params_.height_voxel_dim, 
                                           map_params_.height_voxel_dim);

    float intensity = 0.0f;
    terrain_height_grid_ = std::make_unique<grid_ns::Grid<float>>(
        height_grid_size, intensity, height_grid_origin, height_grid_resolution, 3);
    terrain_grid_occupy_list_.resize(terrain_height_grid_->GetCellNumber());
    INFLATE_N = std::ceil(map_params_.search_radius / map_params_.height_voxel_dim);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);

    kdtree_terrain_clould_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
}

void MapHandler::ResetGripMapCloud() {
    const int N_CELL = world_obs_cloud_grid_->GetCellNumber();
    for (int i=0; i<N_CELL; i++) {
        world_obs_cloud_grid_->GetCell(i)->clear();
        world_free_cloud_grid_->GetCell(i)->clear();
    }
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
}

void MapHandler::ClearPointCell(const Point3D& point) {
    const Eigen::Vector3i psub = world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z);
    std::vector<Eigen::Vector3i> ray_subs;
    world_obs_cloud_grid_->RayTraceSubs(robot_cell_sub_, psub, ray_subs);
    const int H = neighbor_Hnum_ / 2;
    for (const auto& sub : ray_subs) {
        for (int k = -H; k <= H; k++) {
            Eigen::Vector3i csub = sub;
            csub.z() += k;
            const int ind = world_obs_cloud_grid_->Sub2Ind(csub);
            if (!world_obs_cloud_grid_->InRange(csub) || !FARUtil::IsTypeInStack(ind, neighbor_indices_)) continue; 
            world_obs_cloud_grid_->GetCell(ind)->clear();
            world_free_cloud_grid_->GetCell(ind)->clear();
            global_visited_induces_[ind] = 0;
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
                    *cloudOut += *(world_free_cloud_grid_->GetCell(csub));
                } else if (type == CloudType::OBS_CLOUD) {
                    *cloudOut += *(world_obs_cloud_grid_->GetCell(csub));
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
    map_origin.x = ori_robot_pos.x - (map_params_.ceil_length * row_num_) / 2.0f;
    map_origin.y = ori_robot_pos.y - (map_params_.ceil_length * col_num_) / 2.0f;
    map_origin.z = ori_robot_pos.z - (map_params_.ceil_height * level_num_) / 2.0f; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x,map_origin.y,map_origin.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    // set free cloud origin -> centered at current ground
    pointcloud_grid_origin.z() -= FARUtil::vehicle_height;
    world_free_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    is_init_ = true;
    if (FARUtil::IsDebug) ROS_WARN("MH: Global Cloud Map Grid Initialized.");
}

void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);

    robot_cell_sub_ = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    // Get neighbor indices
    neighbor_indices_.clear();
    const int N = neighbor_Lnum_ / 2;
    const int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            for (int k = -H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_indices_.push_back(ind);
                }
            }
        }
    }
    this->SetTerrainHeightGridOrigin(odom_pos);
}

void MapHandler::SetTerrainHeightGridOrigin(const Point3D& robot_pos) {
    // update terrain height grid center
    Eigen::Vector3d grid_origin;
    grid_origin.x() = robot_pos.x - (map_params_.height_voxel_dim * height_dim_) / 2.0f;
    grid_origin.y() = robot_pos.y - (map_params_.height_voxel_dim * height_dim_) / 2.0f;
    grid_origin.z() = 0.0f        - (map_params_.height_voxel_dim * 1.0f) / 2.0f;
    terrain_height_grid_->SetOrigin(grid_origin);
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_indices_) {
        if (world_obs_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind));
    }
}

void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    if (!is_init_) return;
    freeCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_indices_) {
        if (world_free_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *freeCloudOut += *(world_free_cloud_grid_->GetCell(neighbor_ind));
    }
}

void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudIn) {
    if (!is_init_ || obsCloudIn->empty()) return;
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    for (const auto& point : obsCloudIn->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        world_obs_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_obs_modified_list_[ind] = 1;
        global_visited_induces_[ind] = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_obs_modified_list_[i] == 1) FARUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    if (!is_init_ || freeCloudIn->empty()) return;
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        world_free_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_free_modified_list_[ind] = 1;
        global_visited_induces_[ind]  = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_free_cloud_grid_->GetCellNumber(); ++i) {
      if (util_free_modified_list_[i] == 1) FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes, const bool& is_vehicle_height) {
    const float add_h = is_vehicle_height ? FARUtil::vehicle_height : 0.0f;
    const float max_h = FARUtil::robot_pos.z + FARUtil::kTolerZ;
    const float min_h = FARUtil::robot_pos.z - FARUtil::kTolerZ;
    for (const auto& node_ptr : nodes) {
        if (FARUtil::IsFreeNavNode(node_ptr)) continue;
        const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(node_ptr->position.x, 
                                                                                  node_ptr->position.y, 0.0f));
        if (!terrain_height_grid_->InRange(sub)) continue;
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (terrain_grid_occupy_list_[ind] == 0) continue;
        node_ptr->position.z = terrain_height_grid_->GetCell(ind);
        if (node_ptr->position.z < FARUtil::robot_pos.z) node_ptr->position.z += add_h; 
        node_ptr->position.z = std::max(std::min(node_ptr->position.z, max_h), min_h);
    }
}

void MapHandler::AjustCTNodeHeight(const CTNodeStack& ctnodes, const bool& is_vehicle_height) {
    const float add_h = is_vehicle_height ? FARUtil::vehicle_height : 0.0f;
    const float max_h = FARUtil::robot_pos.z + FARUtil::kTolerZ;
    const float min_h = FARUtil::robot_pos.z - FARUtil::kTolerZ;
    for (auto& ctnode_ptr : ctnodes) {
        const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(ctnode_ptr->position.x, 
                                                                                  ctnode_ptr->position.y, 0.0f));
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (!terrain_height_grid_->InRange(sub) || terrain_grid_occupy_list_[ind] == 0) {
            ctnode_ptr->position.z = this->NearestHeightOfPoint(ctnode_ptr->position);
        } else {
            ctnode_ptr->position.z = terrain_height_grid_->GetCell(ind);
            ctnode_ptr->is_ground_associated = true;
        }
        if (ctnode_ptr->position.z < FARUtil::robot_pos.z) ctnode_ptr->position.z += add_h;
        ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, max_h), min_h);
    }
}

float MapHandler::NearestHeightOfPoint(const Point3D& p) {
    // Find the nearest node in graph
    std::vector<int> pIdxK(1);
    std::vector<float> pdDistK(1);
    PCLPoint pcl_p = FARUtil::Point3DToPCLPoint(p);
    if (kdtree_terrain_clould_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
        const PCLPoint terrain_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]];
        return terrain_p.z;
    }
    return p.z;
}

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    PointCloudPtr copy_free_ptr(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*freeCloudIn, *copy_free_ptr);
    for (auto& p : copy_free_ptr->points) {
        p.intensity = p.z;
        p.z = 0.0f;
    }
    FARUtil::FilterCloud(copy_free_ptr, map_params_.height_voxel_dim);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    for (const auto& point : copy_free_ptr->points) {
        Eigen::Vector3i csub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        std::vector<Eigen::Vector3i> subs;
        this->Expansion2D(csub, subs, INFLATE_N);
        for (const auto& sub : subs) {
            if (!terrain_height_grid_->InRange(sub)) continue;
            const int ind = terrain_height_grid_->Sub2Ind(sub);
            if (terrain_grid_occupy_list_[ind] == 0) {
                terrain_height_grid_->GetCell(ind) = point.intensity;
            } else {
                terrain_height_grid_->GetCell(ind) += point.intensity;
            }
            terrain_grid_occupy_list_[ind] += 1;
        }
    }
    const int N = terrain_grid_occupy_list_.size();
    terrainHeightOut->clear();
    for (int i=0; i<N; i++) {
        const int counter = terrain_grid_occupy_list_[i];
        if (counter != 0) {
            Eigen::Vector3d cpos = terrain_height_grid_->Ind2Pos(i);
            terrain_height_grid_->GetCell(i) /= (float)counter;
            cpos.z() = terrain_height_grid_->GetCell(i);
            PCLPoint p = FARUtil::Point3DToPCLPoint(Point3D(cpos));
            terrainHeightOut->points.push_back(p);
        }
    }
    if (terrainHeightOut->empty()) { // set terrain height kdtree
        FARUtil::ClearKdTree(terrainHeightOut, kdtree_terrain_clould_);
    } else {
        kdtree_terrain_clould_->setInputCloud(terrainHeightOut);
    }
}

void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    const int N = neighbor_indices_.size();
    for (int i=0; i<N; i++) {
        const int ind = neighbor_indices_[i];
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        util_remove_check_list_[ind] = 1;
    }
    for (const auto& ind : neighbor_indices_) {
        if (util_remove_check_list_[ind] == 1 && global_visited_induces_[ind] == 1) {
            FARUtil::RemoveOverlapCloud(world_obs_cloud_grid_->GetCell(ind), obsCloud);
        }
    }
}

