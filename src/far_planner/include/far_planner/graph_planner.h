#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_graph.h"

enum ReachVote {
    BLOCK = 0,
    REACH = 1
};

struct GraphPlannerParams {
    GraphPlannerParams() = default;
    float converge_dist;
    float adjust_radius;
    float momentum_dist;
    bool  is_autoswitch;
    int   free_thred;
    int   votes_size;
    int   momentum_thred;
};


class GraphPlanner {
private:
rclcpp::Node::SharedPtr nh_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attemptable_sub_;
GraphPlannerParams gp_params_;
NavNodePtr odom_node_ptr_  = NULL;

// goal related values
NavNodePtr goal_node_ptr_   = NULL;
Point3D origin_goal_pos_    = Point3D(0,0,0);
bool is_use_internav_goal_  = false;
bool command_is_free_nav_   = false;
bool is_goal_in_freespace_  = false;
bool is_terrain_associated_ = false;
bool is_goal_init_;
NodePtrStack current_graph_;
bool is_free_nav_goal_;

// momentum planning values
NodePtrStack recorded_path_;
Point3D next_waypoint_;
int path_momentum_counter_;
bool is_global_path_init_;
float last_waypoint_dist_;
Point3D last_planning_odom_;

// local terrain map for freespace adjustment
Point3D grid_center_ = Point3D(0,0,0);
std::unique_ptr<grid_ns::Grid<char>> free_terrain_grid_;

float PriorityScore(const NavNodePtr& node_ptr);

bool ReconstructPath(const NavNodePtr& goal_node_ptr,
                     const bool& is_free_nav,
                     NodePtrStack& global_path);

bool IsNodeConnectInFree(const NavNodePtr& current_node,
                         const NavNodePtr& neighbor_node);

bool IsValidConnectToGoal(const NavNodePtr& node_ptr, 
                          const NavNodePtr& goal_node_ptr);

NavNodePtr NextNavWaypointFromPath(const NodePtrStack& global_path, const NavNodePtr goal_ptr);

void AttemptStatusCallBack(const std_msgs::msg::Bool::SharedPtr msg);

inline bool IsInvalidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) {
            return true;
        }
    }
    return false;
}

inline void ResetFreeTerrainGridOrigin(const Point3D& p) {
    Eigen::Vector3d grid_origin;
    grid_origin.x() = p.x - (free_terrain_grid_->GetResolution().x() * free_terrain_grid_->GetSize().x()) / 2.0f;
    grid_origin.y() = p.y - (free_terrain_grid_->GetResolution().y() * free_terrain_grid_->GetSize().y()) / 2.0f;
    grid_origin.z() = p.z - (free_terrain_grid_->GetResolution().z() * free_terrain_grid_->GetSize().z()) / 2.0f;
    free_terrain_grid_->SetOrigin(grid_origin);
    grid_center_ = p;
}

/* define inline functions */
inline void InitNodesStates(const NodePtrStack& graph) {
    for (const auto& node_ptr : graph) {
        node_ptr->gscore              = FARUtil::kINF;
        node_ptr->fgscore             = FARUtil::kINF;
        node_ptr->is_traversable      = false;
        node_ptr->is_free_traversable = false;
        node_ptr->parent              = NULL;
        node_ptr->free_parent         = NULL;
    }
}

inline void RecordPathInfo(const NodePtrStack& global_path) {
    if (global_path.size() < 2) {
        if (FARUtil::IsDebug) RCLCPP_ERROR(nh_->get_logger(), "GP: recording path for momontum fails, planning path is empty");
        return;
    }
    recorded_path_         = global_path;
    next_waypoint_         = global_path[1]->position;
    last_waypoint_dist_    = (odom_node_ptr_->position - next_waypoint_).norm();
    is_global_path_init_   = true;
    path_momentum_counter_ = 0;
    last_planning_odom_    = odom_node_ptr_->position;
}

inline bool IsResetBlockStatus(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if (node_ptr->is_odom || (node_ptr->is_near_nodes && (!node_ptr->is_finalized || node_ptr->is_frontier))) return true;
    if (!FARUtil::IsStaticEnv && node_ptr->is_near_nodes) return true;
    const auto it = node_ptr->edge_votes.find(goal_ptr->id);
    if (node_ptr->is_near_nodes && it != node_ptr->edge_votes.end() && int(it->second.size()) < gp_params_.votes_size) {
        return true;
    }
    return false;
}

inline float EulerCost(const NavNodePtr& current_node,
                       const NavNodePtr& neighbor_node) 
{
    return (current_node->position - neighbor_node->position).norm();
}

inline void GoalReset() {
    origin_goal_pos_ = Point3D(0,0,0);
    is_goal_in_freespace_ = false;
    if (goal_node_ptr_ != NULL) {
        if (!is_use_internav_goal_) DynamicGraph::ClearGoalNodeInGraph(goal_node_ptr_);
        else goal_node_ptr_->is_goal = false;
    }
    goal_node_ptr_ = NULL;
}

public:

GraphPlanner() = default;
~GraphPlanner() = default;

void Init(const rclcpp::Node::SharedPtr nh, const GraphPlannerParams& params);


/**
 * Update Global Graph
 * @param vgraph current graph
*/
inline void UpdaetVGraph(const NodePtrStack& vgraph) {current_graph_ = vgraph;};

/**
 * Update Global Graph Traversability Status and gscores
 * @param graph current graph
 * @param odom_node_ptr current odom node ptr
 * @param goal_ptr current goal node ptr
*/
void UpdateGraphTraverability(const NavNodePtr& odom_node_ptr, const NavNodePtr& goal_ptr);

/**
 * Generate path to goal based on traversibility result
 * @param goal_ptr current goal node
 * @param global_path(return) return the global path from odom node position
 * @param _nav_node_ptr(return) current navigation waypoint
 * @param _goal_p(return) goal position after free space adjust 
 * @param _is_fail(return) whether the planner fails to find the path
 * @param _is_succeed(return) whether the vehicle has reached the goal
 * @param _is_free_nav(return) the attemptable navigation status (True)->Non-attempts
 * @return whether or not planning success -> publish a valid path for navigation
*/

bool PathToGoal(const NavNodePtr& goal_ptr,
                NodePtrStack& global_path,
                NavNodePtr&   _nav_node_ptr,
                Point3D&      _goal_p,
                bool&         _is_fails,
                bool&         _is_succeed,
                bool&         _is_free_nav);

/**
 * @brief Update connectivity between goal node and navigation graph
 * @param goal_ptr new create goal node pointer
 * @param updated_graph current updated navigation graph
 */

void UpdateGoalNavNodeConnects(const NavNodePtr& goal_ptr);

/**
 * Graph planner goal update API
 * @param goal goal position
*/ 
void UpdateGoal(const Point3D& goal);


/**
 * @brief Update free terrian grid for re-selecting goal position into free space
 * @param center current center of grid
 * @param obsCloudIn Obstalce point cloud input
 * @param freeCloudIn Free point cloud input
 */
void UpdateFreeTerrainGrid(const Point3D& center,
                           const PointCloudPtr& obsCloudIn, 
                           const PointCloudPtr& freeCloudIn);

/**
 * @brief Adjust goal position based on terrain and polygons
 * @param goal_ptr current goal node pointer
 * @param is_adjust_height whether or not adjust height of goal -> (False if multi layer planning)
 */
void ReEvaluateGoalPosition(const NavNodePtr& goal_ptr, const bool& is_adjust_height);

/**
 * @brief Reset internal values and containers
 */
inline void ResetPlannerInternalValues() {
    goal_node_ptr_ = NULL; 
    odom_node_ptr_ = NULL;
    
    is_goal_init_         = false;
    is_use_internav_goal_ = false;
    is_global_path_init_  = false;
    is_free_nav_goal_     = false;
    is_goal_in_freespace_ = false;
    
    current_graph_.clear(); 
    recorded_path_.clear();
    path_momentum_counter_ = 0;
    last_waypoint_dist_    = 0.0;
    origin_goal_pos_    = Point3D(0,0,0);
    next_waypoint_      = Point3D(0,0,0);
    last_planning_odom_ = Point3D(0,0,0);
}

const NavNodePtr& GetGoalNodePtr() const { return goal_node_ptr_;};

Point3D GetOriginNodePos(const bool& is_adjusted_z) const {
    if (goal_node_ptr_ == NULL) return Point3D(0,0,0);
    if (!is_adjusted_z) return origin_goal_pos_;
    else {
        return Point3D(origin_goal_pos_.x,
                       origin_goal_pos_.y,
                       goal_node_ptr_->position.z);
    }
}

};

#endif