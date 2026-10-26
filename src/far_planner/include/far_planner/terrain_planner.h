#ifndef TERRIAN_PLANNER_H
#define TERRIAN_PLANNER_H

#include <cstdint>

#include "utility.h"
#include "planner_visualizer.h"

struct TerrainPlannerParams {
    TerrainPlannerParams() = default;
    std::string world_frame;
    float radius;
    float voxel_size;
    int   inflate_size;
};

struct TerrainNode {
    TerrainNode() = default;
    int id;
    Point3D position;
    bool is_occupied = false;
    float fscore;
    float gscore;
    std::shared_ptr<TerrainNode> parent;
};

typedef std::shared_ptr<TerrainNode> TerrainNodePtr;
typedef std::vector<TerrainNode> TerrainNodeStack;


class TerrainPlanner {

public:
    TerrainPlanner() = default;
    ~TerrainPlanner() = default;

    void Init(const rclcpp::Node::SharedPtr nh, const TerrainPlannerParams& params);

    void UpdateCenterNode(const NavNodePtr& node_ptr);
    
    void SetLocalTerrainObsCloud(const PointCloudPtr& obsCloudIn, std::uint64_t revision);
    
    bool PlanPathFromPToP(const Point3D& from_p, const Point3D& to_p, PointStack& path);

    inline bool PlanPathFromNodeToNode(const NavNodePtr& node_from, const NavNodePtr& node_to, PointStack& path) {
        return PlanPathFromPToP(node_from->position, node_to->position, path); 
    }

    inline bool IsPointOccupy(const Point3D& p) {
        if (!is_grids_init_) return false;
        const Eigen::Vector3i sub = terrain_grids_->Pos2Sub(p.x, p.y, center_pos_.z);
        if (terrain_grids_->InRange(sub) && terrain_grids_->GetCell(sub)->is_occupied) {
            return true;
        }
        return false;
    }
    
    void VisualPaths();

private:
    rclcpp::Node::SharedPtr nh_;
    TerrainPlannerParams tp_params_;
    int row_num_, col_num_;
    bool is_grids_init_ = false;
    NavNodePtr center_node_prt_ = NULL;
    Point3D center_pos_;
    std::vector<PointStack> viz_path_stack_;

    rclcpp::Publisher<Marker>::SharedPtr local_path_pub_; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_pub_;

    std::unique_ptr<grid_ns::Grid<TerrainNodePtr>> terrain_grids_;
    std::vector<int> occupied_cells_;
    PointCloudPtr local_obs_cloud_;
    std::uint64_t local_terrain_revision_ = 0;
    std::uint64_t last_projected_revision_ = 0;
    bool needs_projection_ = true;

    void ExtractPath(const TerrainNodePtr& end_ptr, PointStack& path);

    void GridVisualCloud();

    inline float EulerCost(const TerrainNodePtr& node_ptr1, const TerrainNodePtr& node_ptr2) {
        return (node_ptr1->position - node_ptr2->position).norm();
    }

    inline void AllocateGridNodes() {
        for (int i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i) = std::make_shared<TerrainNode>();
            terrain_grids_->GetCell(i)->id = i;
        }
    }

    inline void ResetGridsOccupancy() {
        if (!is_grids_init_) return;
        for (const int i : occupied_cells_) {
            terrain_grids_->GetCell(i)->is_occupied = false;
        }
        occupied_cells_.clear();
    }

    inline void ResetGridsPositions() {
        if (!is_grids_init_) return;
        for (int i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i)->position = Point3D(terrain_grids_->Ind2Pos(i));
        }
    }

    inline void ResetGridsPlanStatus() {
        if (!is_grids_init_) return;
        for (int i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i)->fscore = FARUtil::kINF;
            terrain_grids_->GetCell(i)->gscore = FARUtil::kINF;
            terrain_grids_->GetCell(i)->parent = NULL;
        }
    }

    inline Point3D Ind2Point3D(const int& ind) {
        return terrain_grids_->GetCell(ind)->position;
    }

    inline PCLPoint Ind2PCLPoint(const int& ind) {
        return FARUtil::Point3DToPCLPoint(Ind2Point3D(ind));
    }

};


#endif
