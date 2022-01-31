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
    graph_pub_ = nh_.advertise<visibility_graph_msg::Graph>("/robot_vgraph", 5);
    graph_sub_ = nh_.subscribe("/decoded_vgraph", 5, &GraphMsger::GraphCallBack, this);

    global_graph_.clear();
    nodes_cloud_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_graph_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    kdtree_graph_cloud_->setSortedResults(false);
}

void GraphMsger::EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::Graph& graphOut) {
    graphOut.nodes.clear();
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr->connect_nodes.empty() || !IsEncodeType(node_ptr)) continue;
        visibility_graph_msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.position    = FARUtil::Point3DToGeoMsgPoint(node_ptr->position);
        msg_node.id          = node_ptr->id;
        msg_node.FreeType    = static_cast<int>(node_ptr->free_direct);
        msg_node.is_covered  = node_ptr->is_covered;
        msg_node.is_frontier = node_ptr->is_frontier;
        msg_node.is_navpoint = node_ptr->is_navpoint;
        msg_node.is_boundary = node_ptr->is_boundary;
        msg_node.surface_dirs.clear();
        msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.first));
        msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.second));
        // Encode connections
        msg_node.connect_nodes.clear();
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.connect_nodes.push_back(cnode_ptr->id);
        }
        msg_node.poly_connects.clear();
        for (const auto& cnode_ptr : node_ptr->poly_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.poly_connects.push_back(cnode_ptr->id);
        }
        msg_node.contour_connects.clear();
        for (const auto& cnode_ptr : node_ptr->contour_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.contour_connects.push_back(cnode_ptr->id);
        }
        msg_node.trajectory_connects.clear();
        for (const auto& cnode_ptr : node_ptr->trajectory_connects) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.trajectory_connects.push_back(cnode_ptr->id);
        }
        graphOut.nodes.push_back(msg_node);
    }
    graphOut.size = graphOut.nodes.size();
}

void GraphMsger::UpdateGlobalGraph(const NodePtrStack& graph) {
    global_graph_ = graph;
    // Update Graph Node KdTree
    if (global_graph_.empty()) {
        FARUtil::ClearKdTree(nodes_cloud_ptr_, kdtree_graph_cloud_);
    } else {
        nodes_cloud_ptr_->resize(global_graph_.size());
        std::size_t idx = 0;
        for (const auto node_ptr : global_graph_) {
            if (node_ptr->is_odom || FARUtil::IsOutsideGoal(node_ptr)) continue;
            PCLPoint pcl_p = FARUtil::Point3DToPCLPoint(node_ptr->position);
            pcl_p.intensity = node_ptr->id;
            nodes_cloud_ptr_->points[idx] = pcl_p;
            idx ++;
        }
        kdtree_graph_cloud_->setInputCloud(nodes_cloud_ptr_);
    }
    this->PublishGlobalGraph(global_graph_);
}

void GraphMsger::PublishGlobalGraph(const NodePtrStack& graphIn) {
    visibility_graph_msg::Graph graph_msg;
    graph_msg.header.frame_id = gm_params_.frame_id;
    graph_msg.robot_id = gm_params_.robot_id;
    this->EncodeGraph(graphIn, graph_msg);
    graph_pub_.publish(graph_msg);
}

void GraphMsger::GraphCallBack(const visibility_graph_msg::GraphConstPtr& msg) {
    if (msg->nodes.empty()) return;
    NodePtrStack decoded_nodes;
    const std::size_t robot_id = msg->robot_id;
    const visibility_graph_msg::Graph graph_msg = *msg;
    IdxMap nodeIdx_idx_map;
    // Create nav nodes for decoded graph
    decoded_nodes.clear();
    for (std::size_t i=0; i<graph_msg.nodes.size(); i++) {
        const auto node = graph_msg.nodes[i];
        const Point3D node_p = Point3D(node.position.x, node.position.y, node.position.z);
        NavNodePtr nearest_node_ptr = NearestNodePtrOnGraph(node_p, gm_params_.dist_margin);
        if (nearest_node_ptr == NULL || IsMismatchFreeNode(nearest_node_ptr, node)) {
            CreateDecodedNavNode(node, nearest_node_ptr);
            DynamicGraph::AddNodeToGraph(nearest_node_ptr);
        }
        decoded_nodes.push_back(nearest_node_ptr);
        nodeIdx_idx_map.insert({node.id, i});
    }
    // Assign connections with fully connection votes
    std::vector<std::size_t> connect_idxs, poly_idxs, contour_idxs, traj_idxs;
    for (std::size_t i=0; i<graph_msg.nodes.size(); i++) {
        const auto node = graph_msg.nodes[i];
        const NavNodePtr node_ptr = decoded_nodes[i];
        ExtractConnectIdxs(node, connect_idxs, poly_idxs, contour_idxs, traj_idxs);
        // graph connections
        NavNodePtr cnode_ptr = NULL;
        for (const auto& cid : connect_idxs) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::AddEdge(node_ptr, cnode_ptr);
            }
        }
        // poly connections
        for (const auto& cid : poly_idxs) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillPolygonEdgeConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
            }
        }
        // contour connections
        for (const auto& cid : contour_idxs) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillContourConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
            }
        }
        // trajectory connection
        for (const auto& cid : traj_idxs) {
            cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
            if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                DynamicGraph::FillTrajConnect(node_ptr, cnode_ptr);
            }
        }
    }
}

NavNodePtr GraphMsger::NearestNodePtrOnGraph(const Point3D p, const float radius) {
    if (global_graph_.empty()) return NULL;
    // Find the nearest node in graph
    std::vector<int> pIdxK(1);
    std::vector<float> pdDistK(1);
    PCLPoint pcl_p = FARUtil::Point3DToPCLPoint(p);
    if (kdtree_graph_cloud_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
        if (pdDistK[0] < radius) {
            const PCLPoint graph_p = nodes_cloud_ptr_->points[pIdxK[0]];
            const std::size_t node_id = static_cast<std::size_t>(graph_p.intensity);
            return DynamicGraph::MappedNavNodeFromId(node_id);
        }
    }
    return NULL;
}

void GraphMsger::CreateDecodedNavNode(const visibility_graph_msg::Node& vnode, NavNodePtr& node_ptr) {
    const Point3D p = Point3D(vnode.position.x, vnode.position.y, vnode.position.z);
    const bool is_covered  = vnode.is_covered  == 0 ? false : true;
    const bool is_frontier = vnode.is_frontier == 0 ? false : true;
    const bool is_navpoint = vnode.is_navpoint == 0 ? false : true;
    const bool is_boundary = vnode.is_boundary == 0 ? false : true;
    DynamicGraph::CreateNavNodeFromPoint(p, node_ptr, false, is_navpoint, false, is_boundary);
    node_ptr->is_active = false;
    /* Assign relative values */
    node_ptr->is_covered  = is_covered;
    node_ptr->is_frontier = is_frontier;
    DynamicGraph::FillFrontierVotes(node_ptr, is_frontier);
    // positions
    node_ptr->is_finalized = true;
    const std::deque<Point3D> pos_queue(gm_params_.pool_size, p);
    node_ptr->pos_filter_vec = pos_queue;
    const PointPair surf_pair = {Point3D(vnode.surface_dirs[0].x, vnode.surface_dirs[0].y, vnode.surface_dirs[0].z),
                                 Point3D(vnode.surface_dirs[1].x, vnode.surface_dirs[1].y, vnode.surface_dirs[1].z)};
    // surf directions
    node_ptr->free_direct = static_cast<NodeFreeDirect>(vnode.FreeType);
    node_ptr->surf_dirs = surf_pair;
    const std::deque<PointPair> surf_queue(gm_params_.pool_size, surf_pair);
    node_ptr->surf_dirs_vec = surf_queue;
}

void GraphMsger::ExtractConnectIdxs(const visibility_graph_msg::Node& node,
                                    IdxStack& connect_idxs,
                                    IdxStack& poly_idxs,
                                    IdxStack& contour_idxs,
                                    IdxStack& traj_idxs)
{
    connect_idxs.clear(), poly_idxs.clear(), contour_idxs.clear(), traj_idxs.clear();
    for (const auto& cid : node.connect_nodes) {
        connect_idxs.push_back((std::size_t)cid);
    }
    for (const auto& cid : node.poly_connects) {
        poly_idxs.push_back((std::size_t)cid);
    }    
    for (const auto& cid : node.contour_connects) {
        contour_idxs.push_back((std::size_t)cid);
    }
    for (const auto& cid : node.trajectory_connects) {
        traj_idxs.push_back((std::size_t)cid);
    }
}

