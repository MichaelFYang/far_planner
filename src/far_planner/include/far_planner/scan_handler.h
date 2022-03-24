#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "utility.h"

struct ScanHandlerParams {
    ScanHandlerParams() = default;
    float terrain_range;
    float voxel_size;
    float ceil_height;
};

enum GridStatus {
    EMPTY = 0,
    SCAN  = 1,
    OBS   = 2,
    RAY   = 3,
};

class ScanHandler {

public:
    ScanHandler() = default;
    ~ScanHandler() = default;

    void Init(const ScanHandlerParams& params);

    void UpdateRobotPosition(const Point3D& odom_pos);
    
    void SetCurrentScanCloud(const PointCloudPtr& scanCloudIn, const PointCloudPtr& freeCloudIn);

    void SetSurroundObsCloud(const PointCloudPtr& obsCloudIn, 
                             const bool& is_fiWlter_cloud=false);

    void ExtractDyObsCloud(const PointCloudPtr& cloudIn, const PointCloudPtr& dyObsCloudOut);

    void GridVisualCloud(const PointCloudPtr& cloudOut, const GridStatus& type);

    void ReInitGrids();

    inline PCLPoint Ind2PCLPoint(const int& ind) {
        PCLPoint pcl_p;
        Eigen::Vector3d pos = voxel_grids_->Ind2Pos(ind);
        pcl_p.x = pos.x(), pcl_p.y = pos.y(), pcl_p.z = pos.z();
        return pcl_p;
    }

private:
    ScanHandlerParams scan_params_;
    Eigen::Vector3i center_sub_;
    int row_num_, col_num_, level_num_;
    bool is_grids_init_ = false;
    PCLPoint center_p_;
    // Set resolution for Velodyne LiDAR PUCK: https://www.amtechs.co.jp/product/VLP-16-Puck.pdf
    const float ANG_RES_Y = 2.0f/180.0f * M_PI; // vertical resolution 2 degree
    const float ANG_RES_X = 0.5f/180.0f * M_PI; // horizontal resolution 0.5 degree
    std::unique_ptr<grid_ns::Grid<char>> voxel_grids_;

    void SetMapOrigin(const Point3D& ori_robot_pos);
    void SetRayCloud(const Eigen::Vector3i& point_sub);

};


#endif