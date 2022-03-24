/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */


#include "far_planner/scan_handler.h"

/***************************************************************************************/

const char INIT_BIT = char(0); // 0000
const char SCAN_BIT = char(1); // 0001
const char OBS_BIT  = char(2); // 0010
const char RAY_BIT  = char(4); // 0100

void ScanHandler::Init(const ScanHandlerParams& params) {
    scan_params_ = params;
    row_num_ = std::ceil(scan_params_.terrain_range * 2.0f / scan_params_.voxel_size);
    if (row_num_ % 2 == 0) row_num_ ++;
    col_num_ = row_num_;
    level_num_ = std::ceil(scan_params_.ceil_height * 2.0f / scan_params_.voxel_size);
    if (level_num_ % 2 == 0) level_num_ ++;
    Eigen::Vector3i grid_size(row_num_, col_num_, level_num_);
    Eigen::Vector3d grid_origin(0,0,0);
    Eigen::Vector3d grid_resolution(scan_params_.voxel_size, scan_params_.voxel_size, scan_params_.voxel_size);
    voxel_grids_ = std::make_unique<grid_ns::Grid<char>>(grid_size, INIT_BIT, grid_origin, grid_resolution, 3);
    is_grids_init_ = true;
}

void ScanHandler::ReInitGrids() {
    if (!is_grids_init_) return;
    voxel_grids_->ReInitGrid(INIT_BIT);
}

void ScanHandler::UpdateRobotPosition(const Point3D& robot_pos) {
    if (!is_grids_init_) return;
    Eigen::Vector3d grid_origin;
    grid_origin.x() = robot_pos.x - (scan_params_.voxel_size * row_num_) / 2.0f;
    grid_origin.y() = robot_pos.y - (scan_params_.voxel_size * col_num_) / 2.0f;
    grid_origin.z() = robot_pos.z - (scan_params_.voxel_size * level_num_) / 2.0f;
    voxel_grids_->SetOrigin(grid_origin);
    center_sub_ = voxel_grids_->Pos2Sub(Eigen::Vector3d(robot_pos.x, robot_pos.y, robot_pos.z));
    center_p_ = FARUtil::Point3DToPCLPoint(robot_pos);
}

void ScanHandler::SetCurrentScanCloud(const PointCloudPtr& scanCloudIn, const PointCloudPtr& freeCloudIn) {
    if (!is_grids_init_ || scanCloudIn->empty()) return;
    // remove free scan points
    PointCloudPtr copyObsScanCloud(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*scanCloudIn, *copyObsScanCloud);
    FARUtil::RemoveOverlapCloud(copyObsScanCloud, freeCloudIn, true);
    for (const auto& point : copyObsScanCloud->points) { // assign obstacle scan voxels
        const float r = pcl::euclideanDistance(point, center_p_);
        const int L = static_cast<int>(std::ceil((r * ANG_RES_X)/scan_params_.voxel_size/2.0f))+FARUtil::kObsInflate;
        const int N = static_cast<int>(std::ceil((r * ANG_RES_Y)/scan_params_.voxel_size/2.0f));
        Eigen::Vector3i c_sub = voxel_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        for (int i = -L; i <= L; i++) {
            for (int j = -L; j <= L; j++) {
                for (int k = -N; k <= N; k++) {
                    Eigen::Vector3i sub;
                    sub.x() = c_sub.x() + i, sub.y() = c_sub.y() + j, sub.z() = c_sub.z() + k;
                    if (voxel_grids_->InRange(sub)) {
                        const int ind = voxel_grids_->Sub2Ind(sub);
                        voxel_grids_->GetCell(ind) = voxel_grids_->GetCell(ind) | SCAN_BIT;
                    }
                }
            }
        }
    }
    for (const auto& point : scanCloudIn->points) {
        Eigen::Vector3i sub = voxel_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        this->SetRayCloud(sub);
    }
}

void ScanHandler::SetSurroundObsCloud(const PointCloudPtr& obsCloudIn, const bool& is_filter_cloud) {
    if (!is_grids_init_ || obsCloudIn->empty()) return;
    if (is_filter_cloud) FARUtil::FilterCloud(obsCloudIn, scan_params_.voxel_size);
    for (const auto& point : obsCloudIn->points) {
        Eigen::Vector3i sub = voxel_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!voxel_grids_->InRange(sub)) continue;
        const int ind = voxel_grids_->Sub2Ind(sub);
        voxel_grids_->GetCell(ind) = voxel_grids_->GetCell(ind) | OBS_BIT;
    }
}

void ScanHandler::ExtractDyObsCloud(const PointCloudPtr& cloudIn, const PointCloudPtr& dyObsCloudOut) {
    if (!is_grids_init_ || cloudIn->empty()) return;
    dyObsCloudOut->clear();
    for (const auto& point : cloudIn->points) {
        Eigen::Vector3i sub = voxel_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!voxel_grids_->InRange(sub)) continue;
        const int ind = voxel_grids_->Sub2Ind(sub);
        const char cell_c = voxel_grids_->GetCell(ind);
        if ((cell_c | RAY_BIT) == cell_c) {
            dyObsCloudOut->points.push_back(point);
        }
    }
}

void ScanHandler::SetRayCloud(const Eigen::Vector3i& point_sub) {
    Eigen::Vector3i dir_sub = point_sub - center_sub_; 
    if (dir_sub.squaredNorm() < 1.0) return;
    Eigen::Vector3i steps_sub;
    steps_sub.x() = abs(dir_sub.x()), steps_sub.y() = abs(dir_sub.y()), steps_sub.z() = abs(dir_sub.z());
    Eigen::Vector3i sign_sub;
    sign_sub.x() = FARUtil::Signum(dir_sub.x()), sign_sub.y() = FARUtil::Signum(dir_sub.y()), sign_sub.z() = FARUtil::Signum(dir_sub.z());
    if (steps_sub.x() >= std::max(steps_sub.y(), steps_sub.z())) {
        const int N = steps_sub.x();
        const float rx2y = (float)steps_sub.y() / (float)steps_sub.x();
        const float rx2z = (float)steps_sub.z() / (float)steps_sub.x();
        for (int i=1; i<N; i++) {
            Eigen::Vector3i csub = center_sub_;
            csub.x() += i * sign_sub.x();
            csub.y() += (int)std::floor(i*rx2y) * sign_sub.y();
            csub.z() += (int)std::floor(i*rx2z) * sign_sub.z();
            if (!voxel_grids_->InRange(csub)) break;
            const int ind = voxel_grids_->Sub2Ind(csub);
            const char cell_c = voxel_grids_->GetCell(ind); 
            if ((cell_c | SCAN_BIT) == cell_c) break; // hit scan point cell
            voxel_grids_->GetCell(ind) = cell_c | RAY_BIT;
        }
    } else if (steps_sub.y() >= std::max(steps_sub.x(), steps_sub.z())) {
        const int N = steps_sub.y();
        const float ry2x = (float)steps_sub.x() / (float)steps_sub.y();
        const float ry2z = (float)steps_sub.z() / (float)steps_sub.y();
        for (int i=1; i<N; i++) {
            Eigen::Vector3i csub = center_sub_;
            csub.x() += (int)std::floor(i*ry2x) * sign_sub.x();
            csub.y() += i * sign_sub.y();
            csub.z() += (int)std::floor(i*ry2z) * sign_sub.z();
            if (!voxel_grids_->InRange(csub)) break;
            const int ind = voxel_grids_->Sub2Ind(csub);
            const char cell_c = voxel_grids_->GetCell(ind);
            if ((cell_c | SCAN_BIT) == cell_c) break; // hit scan point cell
            voxel_grids_->GetCell(ind) = cell_c | RAY_BIT;
        }
    } else {
        const int N = steps_sub.z();
        const float rz2x = (float)steps_sub.x() / (float)steps_sub.z();
        const float rz2y = (float)steps_sub.y() / (float)steps_sub.z();
        for (int i=1; i<N; i++) {
            Eigen::Vector3i csub = center_sub_;
            csub.x() += (int)std::floor(i*rz2x) * sign_sub.x();
            csub.y() += (int)std::floor(i*rz2y) * sign_sub.y();
            csub.z() += i * sign_sub.z();
            if (!voxel_grids_->InRange(csub)) break;
            const int ind = voxel_grids_->Sub2Ind(csub);
            const char cell_c = voxel_grids_->GetCell(ind);
            if ((cell_c | SCAN_BIT) == cell_c) break; // hit scan point cell
            voxel_grids_->GetCell(ind) = cell_c | RAY_BIT;
        }
    }
}

void ScanHandler::GridVisualCloud(const PointCloudPtr& cloudOut, const GridStatus& type) {
    if (!is_grids_init_) return;
    const int N = voxel_grids_->GetCellNumber();
    cloudOut->clear();
    char RType;
    switch (type) {
        case GridStatus::SCAN:
            RType = SCAN_BIT;
            break;
        case GridStatus::OBS:
            RType = OBS_BIT;
            break;
        case GridStatus::RAY:
            RType = RAY_BIT;
            break;
        default:
            RType = SCAN_BIT;
            break;
    }
    for (int ind=0; ind<N; ind++) {
        const char c_type = voxel_grids_->GetCell(ind);
        if ((c_type | RType) == c_type) {
            cloudOut->points.push_back(this->Ind2PCLPoint(ind));
        }
    }
}