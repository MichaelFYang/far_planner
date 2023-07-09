#ifndef GRAPH_MSGER_H
#define GRAPH_MSGER_H

#include "utility.h"
#include "dynamic_graph.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

struct GraphMsgerParams {
    GraphMsgerParams() = default;
    std::string frame_id;
    int   robot_id;
    int   votes_size;
    int   pool_size;
    float dist_margin;
};


class GraphMsger {
public:
    GraphMsger() = default;
    ~GraphMsger() = default;

    void Init(const rclcpp::Node::SharedPtr nh, const GraphMsgerParams& params);

    void UpdateGlobalGraph(const NodePtrStack& graph);

private:
    rclcpp::Node::SharedPtr nh_;
    GraphMsgerParams gm_params_;
    rclcpp::Publisher<visibility_graph_msg::msg::Graph>::SharedPtr  graph_pub_;
    rclcpp::Subscription<visibility_graph_msg::msg::Graph>::SharedPtr graph_sub_;

    NodePtrStack   global_graph_;
    PointCloudPtr  nodes_cloud_ptr_;
    PointKdTreePtr kdtree_graph_cloud_;
    
    void CreateDecodedNavNode(const visibility_graph_msg::msg::Node& vnode, NavNodePtr& node_ptr);

    inline bool IsEncodeType(const NavNodePtr& node_ptr);

    inline bool IsMismatchFreeNode(const NavNodePtr& nearest_ptr, const visibility_graph_msg::msg::Node& vnode);

    inline NavNodePtr IdToNodePtr(const std::size_t& node_id, const IdxMap& idx_map, const NodePtrStack& node_stack);

    NavNodePtr NearestNodePtrOnGraph(const Point3D p, const float radius);

    void EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut);

    void GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg);

    void PublishGlobalGraph(const NodePtrStack& graphIn);

    bool PublishGraphService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void ExtractConnectIdxs(const visibility_graph_msg::msg::Node& node,
                            IdxStack& connect_idxs,
                            IdxStack& poly_idxs,
                            IdxStack& contour_idxs,
                            IdxStack& traj_idxs);

};

#endif
