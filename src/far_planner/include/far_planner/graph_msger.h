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
    friend class GraphEncodingTestAccess;
public:
    GraphMsger() = default;
    ~GraphMsger() = default;

    void Init(const rclcpp::Node::SharedPtr nh, const GraphMsgerParams& params);

    void UpdateGlobalGraph(const NodePtrStack& graph);

    static bool ValidateGraphMessage(const visibility_graph_msg::msg::Graph& graph,
                                     const std::string& expected_frame, std::string& error);

private:
    rclcpp::Node::SharedPtr nh_;
    GraphMsgerParams gm_params_;
    rclcpp::Publisher<visibility_graph_msg::msg::Graph>::SharedPtr  graph_pub_;
    rclcpp::Subscription<visibility_graph_msg::msg::Graph>::SharedPtr graph_sub_;

    void CreateDecodedNavNode(const visibility_graph_msg::msg::Node& vnode, NavNodePtr& node_ptr);

    static inline bool IsEncodeType(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || !node_ptr->is_finalized || FARUtil::IsOutsideGoal(node_ptr)) {
            return false;
        }
        return true;
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

    static bool EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut);

    void GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg);

};

#endif
