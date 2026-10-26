/*
 * Navigation Graph Decoder
 * Copyright (C) 2021 Fan Yang
 * SPDX-License-Identifier: MIT
 * fanyang2@andrew.cmu.edu,   
 */



#include "graph_decoder/decoder_node.h"
#include <visibility_graph_msg/graph_file_writer.h>
#include <visibility_graph_msg/graph_validation.h>
#include <cmath>
#include <stdexcept>
#include <utility>


/***************************************************************************************/
GraphDecoder::GraphDecoder() {
    this->Init();
}

void GraphDecoder::Init() {
     /* initialize node */
    nh_ = rclcpp::Node::make_shared("graph_decoder_node");
    // Initializing subscribers and publishers in ROS 2:
    
    graph_sub_ = nh_->create_subscription<visibility_graph_msg::msg::Graph>(
        "/robot_vgraph", 
        rclcpp::QoS(5),
        std::bind(&GraphDecoder::GraphCallBack, this, std::placeholders::_1)
    );

    graph_pub_ = nh_->create_publisher<visibility_graph_msg::msg::Graph>("decoded_vgraph", rclcpp::QoS(5));

    graph_viz_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/graph_decoder_viz", rclcpp::QoS(5));

    this->LoadParams();

    save_graph_sub_ = nh_->create_subscription<std_msgs::msg::String>(
        "/save_file_dir", 
        rclcpp::QoS(5),
        std::bind(&GraphDecoder::SaveGraphCallBack, this, std::placeholders::_1)
    );

    read_graph_sub_ = nh_->create_subscription<std_msgs::msg::String>(
        "/read_file_dir", 
        rclcpp::QoS(5),
        std::bind(&GraphDecoder::ReadGraphCallBack, this, std::placeholders::_1)
    );

    request_graph_service_ = nh_->create_service<std_srvs::srv::Trigger>(
        "/request_graph_service",
        std::bind(&GraphDecoder::RequestGraphService, this, std::placeholders::_1, std::placeholders::_2)
    );

    robot_id_ = 0;
    this->ResetGraph(received_graph_);

    // print init complete
    RCLCPP_INFO(nh_->get_logger(), "Graph Decoder Node Initialized");
}


void GraphDecoder::GraphCallBack(const visibility_graph_msg::msg::Graph::SharedPtr msg) {
    std::string error;
    if (!graph_message_validation::ValidateGraph(
            *msg, gd_params_.frame_id,
            [](const auto& node) { return node.freetype; }, error)) {
        RCLCPP_WARN(nh_->get_logger(), "Ignoring malformed graph: %s", error.c_str());
        return;
    }
    NodePtrStack next_graph;
    next_graph.reserve(msg->nodes.size());
    std::unordered_map<std::size_t, std::size_t> nodeIdx_idx_map;
    nodeIdx_idx_map.reserve(msg->nodes.size());
    for (const auto& node : msg->nodes) {
        NavNodePtr temp_node_ptr;
        CreateNavNode(node, temp_node_ptr);
        nodeIdx_idx_map.emplace(node.id, next_graph.size());
        next_graph.push_back(temp_node_ptr);
    }
    const auto assign_validated = [&nodeIdx_idx_map, &next_graph](const auto& ids,
                                      std::vector<std::size_t>& indexes, NodePtrStack& connects) {
        indexes.reserve(ids.size());
        connects.reserve(ids.size());
        for (const auto id : ids) {
            indexes.push_back(id);
            connects.push_back(next_graph[nodeIdx_idx_map.at(id)]);
        }
    };
    for (std::size_t i = 0; i < msg->nodes.size(); ++i) {
        const auto& input = msg->nodes[i];
        const auto& node = next_graph[i];
        assign_validated(input.connect_nodes, node->connect_idxs, node->connect_nodes);
        assign_validated(input.poly_connects, node->poly_idxs, node->poly_connects);
        assign_validated(input.contour_connects, node->contour_idxs, node->contour_connects);
        assign_validated(input.trajectory_connects, node->traj_idxs, node->traj_connects);
    }
    ResetGraph(received_graph_);
    received_graph_.swap(next_graph);
    robot_id_ = msg->robot_id;
    this->VisualizeGraph(received_graph_);
}

void GraphDecoder::AssignConnectNodes(const std::unordered_map<std::size_t, std::size_t>& idxs_map,
                                      const NodePtrStack& graph,
                                      std::vector<std::size_t>& node_idxs,
                                      std::vector<NavNodePtr>& connects)
{
    connects.clear();
    connects.reserve(node_idxs.size());
    std::size_t valid_count = 0;
    for (std::size_t i = 0; i < node_idxs.size(); ++i) {
        const auto it = idxs_map.find(node_idxs[i]);
        if (it != idxs_map.end()) {
            const std::size_t idx = it->second;
            const NavNodePtr cnode_ptr = graph[idx];
            connects.push_back(cnode_ptr);
            node_idxs[valid_count++] = cnode_ptr->id;
        }
    }
    node_idxs.resize(valid_count);
}

void GraphDecoder::CreateNavNode(const visibility_graph_msg::msg::Node& msg, NavNodePtr& node_ptr)
{
    node_ptr = std::make_shared<NavNode>();
    node_ptr->position = Point3D(msg.position.x, msg.position.y, msg.position.z);
    node_ptr->id = msg.id;
    node_ptr->free_direct = static_cast<NodeFreeDirect>(msg.freetype);
    if (msg.surface_dirs.size() != 2) {
        RCLCPP_ERROR(nh_->get_logger(), "node surface directions error.");
        node_ptr->surf_dirs.first = node_ptr->surf_dirs.second = Point3D(0,0,-1);
    } else {
        node_ptr->surf_dirs.first  = Point3D(msg.surface_dirs[0].x,
                                                msg.surface_dirs[0].y,
                                                msg.surface_dirs[0].z);
        node_ptr->surf_dirs.second = Point3D(msg.surface_dirs[1].x,
                                                msg.surface_dirs[1].y,
                                                msg.surface_dirs[1].z);
    }
    node_ptr->is_covered  = msg.is_covered;
    node_ptr->is_frontier = msg.is_frontier;
    node_ptr->is_navpoint = msg.is_navpoint;
    node_ptr->is_boundary = msg.is_boundary;
}


void GraphDecoder::LoadParams() {

    // Declare the parameters
    nh_->declare_parameter("world_frame", "map");
    nh_->declare_parameter("visual_scale_ratio", 1.0f);

    // Retrieve the parameters
    nh_->get_parameter("world_frame", gd_params_.frame_id);
    nh_->get_parameter("visual_scale_ratio", gd_params_.viz_scale_ratio);
    const auto frame_begin = gd_params_.frame_id.find_first_not_of('/');
    if (frame_begin == std::string::npos) {
        throw std::invalid_argument("world_frame must name a nonempty frame");
    }
    gd_params_.frame_id.erase(0, frame_begin);
    if (gd_params_.frame_id.find_first_of(" \t\n\r") != std::string::npos) {
        throw std::invalid_argument("world_frame must not contain whitespace");
    }
    if (!std::isfinite(gd_params_.viz_scale_ratio) || gd_params_.viz_scale_ratio <= 0.0f) {
        throw std::invalid_argument("visual_scale_ratio must be finite and positive");
    }
}

void GraphDecoder::SetMarker(const VizColor& color, 
                             const std::string& ns,
                             const float scale,
                             const float alpha,  
                             Marker& scan_marker) 
{
    scan_marker.header.frame_id = gd_params_.frame_id;
    scan_marker.header.stamp = nh_->now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * gd_params_.viz_scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    this->SetColor(color, alpha, scan_marker);
}

void GraphDecoder::SetColor(const VizColor& color, 
                            const float alpha, 
                            Marker& scan_marker)
{
    std_msgs::msg::ColorRGBA c;
    c.a = alpha;
    if (color == VizColor::RED) {
    c.r = 0.9f, c.g = c.b = 0.f;
    }
    else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
    }
    else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
    }
    else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
    }
    else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
    }
    else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
    }
    else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
    }
    else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
    }
    else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
    }
    else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
    }
    scan_marker.color = c;
}

void GraphDecoder::CreateNavNode(const graph_file::NodeRecord& record, NavNodePtr& node_ptr) {
    node_ptr = std::make_shared<NavNode>();
    node_ptr->id = record.id;
    node_ptr->free_direct = static_cast<NodeFreeDirect>(record.free_type);
    const auto& c = record.coordinates;
    node_ptr->position = Point3D(c[0], c[1], c[2]);
    node_ptr->surf_dirs.first = Point3D(c[3], c[4], c[5]);
    node_ptr->surf_dirs.second = Point3D(c[6], c[7], c[8]);
    node_ptr->is_covered = record.flags[0];
    node_ptr->is_frontier = record.flags[1];
    node_ptr->is_navpoint = record.flags[2];
    node_ptr->is_boundary = record.flags[3];
    node_ptr->connect_idxs.assign(record.connections[0].begin(), record.connections[0].end());
    node_ptr->poly_idxs.assign(record.connections[1].begin(), record.connections[1].end());
    node_ptr->contour_idxs.assign(record.connections[2].begin(), record.connections[2].end());
    node_ptr->traj_idxs.assign(record.connections[3].begin(), record.connections[3].end());
}

void GraphDecoder::EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::msg::Graph& graphOut) {
    graphOut.nodes.clear();
    graphOut.nodes.reserve(graphIn.size());
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        visibility_graph_msg::msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.header.stamp = graphOut.header.stamp;
        msg_node.position    = ToGeoMsgP(node_ptr->position);
        msg_node.id          = node_ptr->id;
        msg_node.freetype    = static_cast<int>(node_ptr->free_direct);
        msg_node.is_covered  = node_ptr->is_covered;
        msg_node.is_frontier = node_ptr->is_frontier;
        msg_node.is_navpoint = node_ptr->is_navpoint;
        msg_node.is_boundary = node_ptr->is_boundary;
        msg_node.surface_dirs.clear();
        msg_node.surface_dirs.push_back(ToGeoMsgP(node_ptr->surf_dirs.first));
        msg_node.surface_dirs.push_back(ToGeoMsgP(node_ptr->surf_dirs.second));
        // Encode connections
        msg_node.connect_nodes.clear();
        for (const auto& cid : node_ptr->connect_idxs) {
            msg_node.connect_nodes.push_back(cid);
        }
        msg_node.poly_connects.clear();
        for (const auto& cid : node_ptr->poly_idxs) {
            msg_node.poly_connects.push_back(cid);
        }
        msg_node.contour_connects.clear();
        for (const auto& cid : node_ptr->contour_idxs) {
            msg_node.contour_connects.push_back(cid);
        }
        msg_node.trajectory_connects.clear();
        for (const auto& cid : node_ptr->traj_idxs) {
            msg_node.trajectory_connects.push_back(cid);
        }
        graphOut.nodes.push_back(std::move(msg_node));
    }
    graphOut.size = graphOut.nodes.size();
}

void GraphDecoder::ReadGraphCallBack(const std_msgs::msg::String::SharedPtr msg) {
    const std::string file_path = msg->data;
    if (file_path.empty()) return;
    std::ifstream graph_file(file_path);
    if (!graph_file) {
        RCLCPP_ERROR(nh_->get_logger(), "Cannot open graph file: %s", file_path.c_str());
        return;
    }
    std::vector<graph_file::NodeRecord> records;
    try {
        records = graph_file::ParseGraph(graph_file);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(nh_->get_logger(), "Invalid graph file %s: %s", file_path.c_str(), error.what());
        return;
    }
    NodePtrStack loaded_graph;
    loaded_graph.reserve(records.size());
    std::unordered_map<std::size_t, std::size_t> nodeIdx_idx_map;
    nodeIdx_idx_map.reserve(records.size());
    for (const auto& record : records) {
        NavNodePtr node;
        CreateNavNode(record, node);
        nodeIdx_idx_map.emplace(node->id, loaded_graph.size());
        loaded_graph.push_back(node);
    }
    for (const auto& node_ptr : loaded_graph) { // add connections to nodes
        AssignConnectNodes(nodeIdx_idx_map, loaded_graph, node_ptr->connect_idxs, node_ptr->connect_nodes);
        AssignConnectNodes(nodeIdx_idx_map, loaded_graph, node_ptr->poly_idxs, node_ptr->poly_connects);
        AssignConnectNodes(nodeIdx_idx_map, loaded_graph, node_ptr->contour_idxs, node_ptr->contour_connects);
        AssignConnectNodes(nodeIdx_idx_map, loaded_graph, node_ptr->traj_idxs, node_ptr->traj_connects);
    }
    this->VisualizeGraph(loaded_graph);
    visibility_graph_msg::msg::Graph graph_msg;
    ConvertGraphToMsg(loaded_graph, graph_msg);
    graph_pub_->publish(graph_msg);
    ResetGraph(loaded_graph);
}

void GraphDecoder::SaveGraphCallBack(const std_msgs::msg::String::SharedPtr msg) {
    if (received_graph_.empty()) return;
    const std::string file_path = msg->data;
    if (file_path.empty()) return;
    try {
        graph_file::WriteGraphFile(file_path, received_graph_);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(nh_->get_logger(), "Cannot save graph file %s: %s", file_path.c_str(), error.what());
    }
}

bool GraphDecoder::RequestGraphService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    
    res->success = false;
    visibility_graph_msg::msg::Graph graph_msg; 
    ConvertGraphToMsg(received_graph_, graph_msg);
    graph_pub_->publish(graph_msg);
    res->message = "Decoded graph published; Total graph size: " + std::to_string(graph_msg.size);
    res->success = true;
    return true;
}

void GraphDecoder::VisualizeGraph(const NodePtrStack& graphIn) {
    if (graph_viz_pub_->get_subscription_count() == 0) return;
    MarkerArray graph_marker_array;
    graph_marker_array.markers.reserve(12);
    Marker nav_node_marker, covered_node_marker, frontier_node_marker, internav_node_marker, edge_marker, poly_edge_marker, 
           contour_edge_marker, free_edge_marker, traj_edge_marker, boundary_edge_marker, corner_surf_marker, corner_helper_marker;
    nav_node_marker.type       = Marker::SPHERE_LIST;
    covered_node_marker.type   = Marker::SPHERE_LIST;
    internav_node_marker.type  = Marker::SPHERE_LIST;
    frontier_node_marker.type  = Marker::SPHERE_LIST;
    edge_marker.type           = Marker::LINE_LIST;
    poly_edge_marker.type      = Marker::LINE_LIST;
    free_edge_marker.type      = Marker::LINE_LIST;
    boundary_edge_marker.type  = Marker::LINE_LIST;
    contour_edge_marker.type   = Marker::LINE_LIST;
    traj_edge_marker.type      = Marker::LINE_LIST;
    corner_surf_marker.type    = Marker::LINE_LIST;
    corner_helper_marker.type  = Marker::CUBE_LIST;
    this->SetMarker(VizColor::WHITE,   "global_vertex",     0.5f,  0.5f,  nav_node_marker);
    this->SetMarker(VizColor::BLUE,    "freespace_vertex",  0.5f,  0.8f,  covered_node_marker);
    this->SetMarker(VizColor::YELLOW,  "trajectory_vertex", 0.5f,  0.8f,  internav_node_marker);
    this->SetMarker(VizColor::ORANGE,  "frontier_vertex",   0.5f,  0.8f,  frontier_node_marker);
    this->SetMarker(VizColor::WHITE,   "global_vgraph",     0.1f,  0.2f,  edge_marker);
    this->SetMarker(VizColor::EMERALD, "freespace_vgraph",  0.1f,  0.2f,  free_edge_marker);
    this->SetMarker(VizColor::EMERALD, "visibility_edge",   0.1f,  0.2f,  poly_edge_marker);
    this->SetMarker(VizColor::RED,     "polygon_edge",      0.15f, 0.2f,  contour_edge_marker);
    this->SetMarker(VizColor::GREEN,   "trajectory_edge",   0.1f,  0.5f,  traj_edge_marker);
    this->SetMarker(VizColor::ORANGE,  "boundary_edge",     0.2f,  0.25f, boundary_edge_marker);
    this->SetMarker(VizColor::YELLOW,  "vertex_angle",      0.15f, 0.75f, corner_surf_marker);
    this->SetMarker(VizColor::YELLOW,  "angle_direct",      0.25f, 0.75f, corner_helper_marker);
    /* Lambda Function */
    auto Draw_Edge = [&](const NavNodePtr& node_ptr) {
        geometry_msgs::msg::Point p1, p2;
        p1 = ToGeoMsgP(node_ptr->position);
        for (const auto& cnode : node_ptr->connect_nodes) {
            p2 = ToGeoMsgP(cnode->position);
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
        }
        for (const auto& cnode : node_ptr->poly_connects) {
            p2 = ToGeoMsgP(cnode->position);
            poly_edge_marker.points.push_back(p1);
            poly_edge_marker.points.push_back(p2);
            if (node_ptr->is_covered && cnode->is_covered) {
                free_edge_marker.points.push_back(p1);
                free_edge_marker.points.push_back(p2);
            }
        }
        // contour edges
        for (const auto& ct_cnode : node_ptr->contour_connects) {
            p2 = ToGeoMsgP(ct_cnode->position);
            contour_edge_marker.points.push_back(p1);
            contour_edge_marker.points.push_back(p2);
            if (node_ptr->is_boundary && ct_cnode->is_boundary) {
                boundary_edge_marker.points.push_back(p1);
                boundary_edge_marker.points.push_back(p2);
            }
        }
        // inter navigation trajectory connections
        if (node_ptr->is_navpoint) {
            for (const auto& tj_cnode : node_ptr->traj_connects) {
                p2 = ToGeoMsgP(tj_cnode->position);
                traj_edge_marker.points.push_back(p1);
                traj_edge_marker.points.push_back(p2);
            }
        }
    };
    auto Draw_Surf_Dir = [&](const NavNodePtr& node_ptr) {
        geometry_msgs::msg::Point p1, p2, p3;
        p1 = ToGeoMsgP(node_ptr->position);
        Point3D end_p;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            end_p = node_ptr->position + node_ptr->surf_dirs.first * gd_params_.viz_scale_ratio;
            p2 = ToGeoMsgP(end_p);
            corner_surf_marker.points.push_back(p1);
            corner_surf_marker.points.push_back(p2);
            corner_helper_marker.points.push_back(p2);
            end_p = node_ptr->position + node_ptr->surf_dirs.second * gd_params_.viz_scale_ratio;
            p3 = ToGeoMsgP(end_p);
            corner_surf_marker.points.push_back(p1);
            corner_surf_marker.points.push_back(p3);
            corner_helper_marker.points.push_back(p3);
        }
    };
    std::size_t idx = 0;
    const std::size_t graph_size = graphIn.size();
    nav_node_marker.points.resize(graph_size);
    for (const auto& nav_node_ptr : graphIn) {
        if (nav_node_ptr == NULL) {
            continue;
        }
        const geometry_msgs::msg::Point cpoint = ToGeoMsgP(nav_node_ptr->position);
        nav_node_marker.points[idx] = cpoint;
        if (nav_node_ptr->is_navpoint) {
            internav_node_marker.points.push_back(cpoint);
        }
        if (nav_node_ptr->is_covered) {
            covered_node_marker.points.push_back(cpoint);
        }
        if (nav_node_ptr->is_frontier) {
            frontier_node_marker.points.push_back(cpoint);
        }
        Draw_Edge(nav_node_ptr);
        Draw_Surf_Dir(nav_node_ptr);
        idx ++;    
    } 
    nav_node_marker.points.resize(idx);
    graph_marker_array.markers.push_back(std::move(nav_node_marker));
    graph_marker_array.markers.push_back(std::move(covered_node_marker));
    graph_marker_array.markers.push_back(std::move(internav_node_marker));
    graph_marker_array.markers.push_back(std::move(frontier_node_marker));
    graph_marker_array.markers.push_back(std::move(edge_marker));
    graph_marker_array.markers.push_back(std::move(poly_edge_marker));
    graph_marker_array.markers.push_back(std::move(boundary_edge_marker));
    graph_marker_array.markers.push_back(std::move(free_edge_marker));
    graph_marker_array.markers.push_back(std::move(contour_edge_marker));
    graph_marker_array.markers.push_back(std::move(traj_edge_marker));
    graph_marker_array.markers.push_back(std::move(corner_surf_marker));
    graph_marker_array.markers.push_back(std::move(corner_helper_marker));
    graph_viz_pub_->publish(graph_marker_array);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<GraphDecoder> gd_node;
  try {
    gd_node = std::make_shared<GraphDecoder>();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("graph_decoder_node"), "Decoder startup error: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::spin(gd_node->GetNodeHandle());
  rclcpp::shutdown();
  return 0;
}
