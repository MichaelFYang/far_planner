#ifndef GRAPH_MSGER_H
#define GRAPH_MSGER_H

#include "utility.h"

struct GraphMsgerParams {
    GraphMsgerParams() = default;
    std::string frame_id;
    int robot_id;
};


class GraphMsger {
public:
    GraphMsger() = default;
    ~GraphMsger() = default;

    void Init(const ros::NodeHandle& nh, const GraphMsgerParams& params);

    void UpdateGlobalGraph(const NodePtrStack& graph) {global_graph_ = graph;};

private:
    ros::NodeHandle nh_;
    GraphMsgerParams gm_params_;
    ros::Publisher graph_pub_;
    ros::ServiceServer request_graph_service_;

    NodePtrStack global_graph_;
    nav_graph_msg::Graph nav_graph_;

    inline bool IsEncodeType(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || FARUtil::IsOutsideGoal(node_ptr)) {
            return false;
        }
        return true;
    }

    void EncodeGraph(const NodePtrStack& graphIn, nav_graph_msg::Graph& graphOut);

    bool RequestGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);



};


#endif