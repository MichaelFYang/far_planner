#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float ceil_length;
    float ceil_height;
    float grid_max_length;
    float grid_max_height;
    // local terrain height map
    float search_radius;
    float height_voxel_dim;
};

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    void AdjustNodesHeight(const NodePtrStack& nodes, const bool& is_vehicle_height=true);

    void AjustCTNodeHeight(const CTNodeStack& ctnodes, const bool& is_vehicle_height=true);

    /** Update global cloud grid with incoming clouds 
     * @param CloudIn incoming cloud ptr
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudIn);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param center the position of the grid that want to extract
     * @param cloudOut output cloud ptr
     * @param type choose free or obstacle cloud for extraction
     * @param is_large whether or not using the surrounding cells
    */
    void GetCloudOfPoint(const Point3D& center, 
                         const PointCloudPtr& CloudOut, 
                         const CloudType& type,
                         const bool& is_large);

    /**
     * Get neihbor ceils center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor ceils center positions
     * @param occupancy_centers[out] occupanied ceils center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();

    /**
     * @brief Clear the cell that the given point located
     * @param point Give point location
     */
    void ClearPointCell(const Point3D& point);

private:

    MapHandlerParams map_params_;
    int row_num_, col_num_, level_num_, neighbor_Lnum_, neighbor_Hnum_, height_dim_;
    Point3D neghbor_ceils_origin_;
    Eigen::Vector3i robot_cell_sub_;
    int INFLATE_N;
    bool is_init_ = false;
    PointKdTreePtr kdtree_terrain_clould_;

    void SetTerrainHeightGridOrigin(const Point3D& robot_pos);

    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n=1) {
        subs.clear();
        for (int ix=-n; ix<=n; ix++) {
            for (int iy=-n; iy<=n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub); 
            }
        }
    }

    float NearestHeightOfPoint(const Point3D& p);

    std::vector<int> neighbor_indices_; // Surrounding neighbor cloud grid indices stack

    std::vector<int> global_visited_induces_;
    std::vector<int> util_obs_modified_list_;
    std::vector<int> util_free_modified_list_;
    std::vector<int> util_remove_check_list_;
    std::vector<int> terrain_grid_occupy_list_;
    
    std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;
    std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;
    std::unique_ptr<grid_ns::Grid<float>> terrain_height_grid_;
 
};

#endif