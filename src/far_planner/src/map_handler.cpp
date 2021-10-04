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
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.ceil_length) + 2; 
    neighbor_Hnum_ = 5; 
    if (level_num_ % 2 == 0) level_num_ ++;         // force to odd number, robot will be at center
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++; // force to odd number
    if (neighbor_Hnum_ % 2 == 0) neighbor_Hnum_ ++; // force to odd number
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
    map_origin_.x = ori_robot_pos.x - (map_params_.ceil_length * row_num_) / 2.0f;
    map_origin_.y = ori_robot_pos.y - (map_params_.ceil_length * col_num_) / 2.0f;
    map_origin_.z = ori_robot_pos.z - (map_params_.ceil_height * level_num_) / 2.0f; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin_.x,map_origin_.y,map_origin_.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
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
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i neighbor_sub;
                neighbor_sub.x() = robot_cell_sub_.x() + i;
                neighbor_sub.y() = robot_cell_sub_.y() + j;
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_indices_.push_back(ind);
                }
            }
        }
    }
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
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_free_modified_list_[i] == 1) FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
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

