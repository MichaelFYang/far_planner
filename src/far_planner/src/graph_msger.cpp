/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/graph_msger.h"

/***************************************************************************************/

void GraphMsger::Init(const ros::NodeHandle& nh, const GraphMsgerParams& params) {
    nh_ = nh;
    gm_params_ = params;
    graph_pub_ = nh_.advertise<nav_graph_msg::Graph>("/planner_nav_graph", 5);
    request_graph_service_ = nh_.advertiseService("/request_nav_graph", &GraphMsger::RequestGraphService, this);

    global_graph_.clear();
    nav_graph_.nodes.clear();
}

void GraphMsger::EncodeGraph(const NodePtrStack& graphIn, nav_graph_msg::Graph& graphOut) {
    graphOut.nodes.clear();
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr->connect_nodes.empty() || !IsEncodeType(node_ptr)) continue;
        nav_graph_msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.position.x = node_ptr->position.x;
        msg_node.position.y = node_ptr->position.y;
        msg_node.position.z = node_ptr->position.z;
        msg_node.id = node_ptr->id;
        msg_node.connect_nodes.clear();
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.connect_nodes.push_back(cnode_ptr->id);
        }
        graphOut.nodes.push_back(msg_node);
    }
    graphOut.size = graphOut.nodes.size();
}

bool GraphMsger::RequestGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (global_graph_.empty()) return false;
    nav_graph_.header.frame_id = gm_params_.frame_id;
    nav_graph_.robot_id = gm_params_.robot_id;
    this->EncodeGraph(global_graph_, nav_graph_);
    graph_pub_.publish(nav_graph_);
    res.message = "Grobal Graph has been shared.";
    res.success = true;
    ROS_INFO_STREAM("GM: Graph of robot " <<gm_params_.robot_id <<" has been published, total number of nodes: " << nav_graph_.size);
    return res.success;
}


