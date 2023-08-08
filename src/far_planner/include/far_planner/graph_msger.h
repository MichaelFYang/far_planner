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

    inline bool IsEncodeType(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || !node_ptr->is_finalized || FARUtil::IsOutsideGoal(node_ptr)) {
            return false;
        }
        return true;
    }

    inline bool IsMismatchFreeNode(const NavNodePtr& nearest_ptr, const visibility_graph_msg::msg::Node& vnode) {
        if (nearest_ptr == NULL) return false;
        const bool is_navpoint = vnode.is_navpoint == 0 ? false : true;
        const bool is_boundary = vnode.is_boundary == 0 ? false : true;
        if (nearest_ptr->is_navpoint != is_navpoint || nearest_ptr->is_boundary != is_boundary) return true;
        return false;
    }
    

    inline NavNodePtr IdToNodePtr(const std::size_t& node_id, const IdxMap& idx_map, const NodePtrStack& node_stack) {
        const auto it = idx_map.find(node_id);
        if (it != idx_map.end()) {
            const std::size_t idx = it->second;
            if (idx < node_stack.size()) {
                return node_stack[idx];
            }
        }
        return NULL;
    }

    NavNodePtr NearestNodePtrOnGraph(const Point3D p, const float radius);

    void EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut);

    void GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg);

    void PublishGlobalGraph(const NodePtrStack& graphIn);

    void ExtractConnectIdxs(const visibility_graph_msg::msg::Node& node,
                            IdxStack& connect_idxs,
                            IdxStack& poly_idxs,
                            IdxStack& contour_idxs,
                            IdxStack& traj_idxs);

};

#endif
