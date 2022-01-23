#ifndef GRAPH_DECODER_H
#define GRAPH_DECODER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <std_msgs/String.h>
#include <visibility_graph_msg/Graph.h>
#include <visibility_graph_msg/Node.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "graph_decoder/point_struct.h"

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

typedef std::pair<Point3D, Point3D> PointPair;

enum VizColor {
    RED = 0,
    ORANGE = 1,
    BLACK = 2,
    YELLOW = 3,
    BLUE = 4,
    GREEN = 5,
    EMERALD = 6,
    WHITE = 7,
    MAGNA = 8,
    PURPLE = 9
};

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3
};

struct NavNode {
    NavNode() = default;
    std::size_t id;
    Point3D position;
    NodeFreeDirect free_direct;
    PointPair surf_dirs;
    bool is_covered;
    bool is_frontier;
    bool is_navpoint;
    bool is_boundary;
    std::vector<std::size_t> connect_idxs;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;

    std::vector<std::size_t> poly_idxs;
    std::vector<std::shared_ptr<NavNode>> poly_connects;

    std::vector<std::size_t> contour_idxs;
    std::vector<std::shared_ptr<NavNode>> contour_connects;

    std::vector<std::size_t> traj_idxs;
    std::vector<std::shared_ptr<NavNode>> traj_connects;
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::vector<NavNodePtr> NodePtrStack;

struct GraphDecoderParams {
    GraphDecoderParams() = default;
    std::string frame_id;
    float viz_scale_ratio;
};

class GraphDecoder {
public:
    GraphDecoder() = default;
    ~GraphDecoder() = default;

    void Init();
    void Loop();

private:
    ros::NodeHandle nh;
    ros::Subscriber graph_sub_;
    ros::Subscriber save_graph_sub_, read_graph_sub_;
    ros::Publisher  graph_pub_, graph_viz_pub_;

    ros::ServiceServer request_graph_service_;
    GraphDecoderParams gd_params_;
    NodePtrStack received_graph_;
    MarkerArray graph_marker_array_;
    std::size_t robot_id_;

    void LoadParmas();

    void SetMarker(const VizColor& color, 
                   const std::string& ns,
                   const float scale,
                   const float alpha, 
                   Marker& scan_marker);
    
    void SetColor(const VizColor& color, const float alpha, Marker& scan_marker);

    void GraphCallBack(const visibility_graph_msg::GraphConstPtr& msg);

    void EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::Graph& graphOut);

    void SaveGraphCallBack(const std_msgs::StringConstPtr& msg);

    void ReadGraphCallBack(const std_msgs::StringConstPtr& msg);

    bool SaveGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool ReadGraphFromFile(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool RequestGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res); 

    void VisualizeGraph(const NodePtrStack& graphIn);

    void CreateNavNode(std::string str, NavNodePtr& node_ptr);

    void CreateNavNode(const visibility_graph_msg::Node& msg, NavNodePtr& node_ptr);

    void AssignConnectNodes(const std::unordered_map<std::size_t, std::size_t>& idxs_map,
                            const NodePtrStack& graph,
                            std::vector<std::size_t>& node_idxs,
                            std::vector<NavNodePtr>& connects);

    template <typename Point>
    geometry_msgs::Point inline ToGeoMsgP(const Point& p) {
        geometry_msgs::Point geo_p;
        geo_p.x = p.x;
        geo_p.y = p.y; 
        geo_p.z = p.z;
        return geo_p;
    }

    template <typename T>
    bool IsTypeInStack(const T& elem, const std::vector<T>& T_stack) {
        if (std::find(T_stack.begin(), T_stack.end(), elem) != T_stack.end()) {
            return true;
        }
        return false;
    }

    inline void ResetGraph(NodePtrStack& graphOut) {
        graphOut.clear();
    }

    inline void ConvertGraphToMsg(const NodePtrStack& graph, visibility_graph_msg::Graph& graph_msg) {
        graph_msg.header.frame_id = gd_params_.frame_id;
        graph_msg.header.stamp = ros::Time::now();
        graph_msg.robot_id = robot_id_;
        EncodeGraph(graph, graph_msg);
    }

    inline bool AddNodePtrToGraph(const NavNodePtr& node_ptr, NodePtrStack& graphOut) {
        if (node_ptr != NULL) {
            graphOut.push_back(node_ptr);
            return true;
        }
        return false;
    }
    
};


#endif