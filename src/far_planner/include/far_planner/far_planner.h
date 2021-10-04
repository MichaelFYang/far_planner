#ifndef FAR_PLANNER_H
#define FAR_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"


struct FARMasterParams {
    FARMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float voxel_dim;
    float sensor_range;
    float terrain_range;
    float waypoint_project_dist;
    float main_run_freq;
    float viz_ratio;
    bool  is_visual_opencv;
    bool  is_static_env;
    bool  is_debug_output;
    bool  is_attempt_autoswitch;
    std::string world_frame;
};

class FARMaster {
public:
    FARMaster() = default;
    ~FARMaster() = default;

    void Init(); // ROS initialization
    void Loop(); // Main Loop Function

private:
    ros::NodeHandle nh;
    ros::Subscriber reset_graph_sub_, joy_command_sub_, update_command_sub_;
    ros::Subscriber odom_sub_, terrain_sub_, terrain_local_sub_, scan_sub_, waypoint_sub_;
    ros::Publisher  goal_pub_, runtime_pub_;
    ros::Publisher  obs_world_pub_, new_PCL_pub_;
    ros::Publisher  dynamic_obs_pub_, surround_free_debug_, surround_obs_debug_, scan_grid_debug_;

    ros::Timer planning_event_;
    std_msgs::Float32 runtimer_;

    Point3D robot_pos_, robot_heading_, nav_heading_, nav_goal_;

    bool is_reset_env_, is_stop_update_;

    geometry_msgs::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_scan_init_, is_odom_init_, is_planner_running_;
    bool is_graph_init_;

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr local_terrain_ptr_;
    PointCloudPtr scan_grid_ptr_;

    NavNodePtr odom_node_ptr_ = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack near_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;
    std::vector<PointStack> realworld_contour_;

    tf::TransformListener* tf_listener_;

    /* module objects */
    ContourDetector contour_detector_;
    DynamicGraph graph_manager_;
    DPVisualizer planner_viz_;
    GraphPlanner graph_planner_;
    ContourGraph contour_graph_;
    MapHandler map_handler_;
    ScanHandler scan_handler_;
    GraphMsger graph_msger_;

    /* ROS Params */
    FARMasterParams      master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;
    
    void LoadROSParams();

    void ResetEnvironmentAndGraph();

    void PlanningCallBack(const ros::TimerEvent& event);
    
    void PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                       const PointCloudPtr& cloudOut);

    Point3D ProjectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint);

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);

    inline void ResetGraphCallBack(const std_msgs::EmptyConstPtr& msg) {
        is_reset_env_ = true;
    }

    inline void JoyCommandCallBack(const sensor_msgs::JoyConstPtr& msg) {
        if (msg->buttons[4] > 0.5) {
            is_reset_env_ = true;
        }
    } 

    inline void UpdateCommandCallBack(const std_msgs::Bool& msg) {
        if (is_stop_update_ && msg.data) {
            if (FARUtil::IsDebug) ROS_WARN("FARMaster: Resume visibility graph update.");
            is_stop_update_ = !msg.data;
        }
        if (!is_stop_update_ && !msg.data) {
            if (FARUtil::IsDebug) ROS_WARN("FARMaster: Stop visibility graph update.");
            is_stop_update_ = !msg.data;
        }   
    }

    void ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void WaypointCallBack(const geometry_msgs::PointStamped& route_goal);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn, 
                                   const PointCloudPtr& dyObsCloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        near_nav_graph_.clear();
        realworld_contour_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL; 
        is_cloud_init_      = false; 
        is_odom_init_       = false; 
        is_scan_init_       = false;
        is_planner_running_ = false;  
        is_graph_init_      = false; 
        ClearTempMemory();
    }
};

#endif