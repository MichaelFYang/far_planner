#ifndef GRAPH_DECODER_H
#define GRAPH_DECODER_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <std_msgs/msg/string.hpp>
#include <visibility_graph_msg/msg/graph.hpp>
#include <visibility_graph_msg/msg/node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "graph_decoder/point_struct.h"

typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;

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
    GraphDecoder();
    ~GraphDecoder() = default;

    void Init(); // ROS init
    rclcpp::Node::SharedPtr GetNodeHandle() { return nh_; }

    void Loop();

private:
    rclcpp::Node::SharedPtr nh_;

    rclcpp::Subscription<visibility_graph_msg::msg::Graph>::SharedPtr graph_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_graph_sub_, read_graph_sub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr graph_viz_pub_;
    rclcpp::Publisher<visibility_graph_msg::msg::Graph>::SharedPtr graph_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_graph_service_;

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

    // Topic Callbacks
    void GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg);

    void EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut);

    void SaveGraphCallBack(const std_msgs::msg::String::SharedPtr msg);

    void ReadGraphCallBack(const std_msgs::msg::String::SharedPtr msg);

    // Service Callbacks
    bool RequestGraphService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);


    void VisualizeGraph(const NodePtrStack& graphIn);

    void CreateNavNode(std::string str, NavNodePtr& node_ptr);

    void CreateNavNode(const visibility_graph_msg::msg::Node& msg, NavNodePtr& node_ptr);

    void AssignConnectNodes(const std::unordered_map<std::size_t, std::size_t>& idxs_map,
                            const NodePtrStack& graph,
                            std::vector<std::size_t>& node_idxs,
                            std::vector<NavNodePtr>& connects);

    template <typename Point>
    geometry_msgs::msg::Point inline ToGeoMsgP(const Point& p) {
        geometry_msgs::msg::Point geo_p;
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

    inline void ConvertGraphToMsg(const NodePtrStack& graph, visibility_graph_msg::msg::Graph& graph_msg) {
        graph_msg.header.frame_id = gd_params_.frame_id;
        graph_msg.header.stamp = nh_->now();
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