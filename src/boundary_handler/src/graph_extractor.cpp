/*
 * Boundary Graph Extractor
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "boundary_handler/graph_extractor.h"
#include "boundary_handler/intersection.h"

/***************************************************************************************/


void GraphExtractor::Init() {
    /* initialize subscriber and publisher */
    graph_viz_pub_ = nh.advertise<MarkerArray>("/graph_decoder_viz",5);
    viz_node_pub_  = nh.advertise<Marker>("/free_p_viz",5);
    this->LoadParmas();
    robot_id_ = 0;
    boundary_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    extracted_polys_.clear(), extracted_graph_.clear();
    this->ReadTrajFile(free_p_);
}


void GraphExtractor::LoadParmas() {
    const std::string prefix = "/boundary_handler/";
    nh.param<std::string>(prefix + "world_frame", ge_params_.frame_id, "map");
    std::string folder_path;
    nh.param<std::string>(prefix + "folder_path", folder_path, "/home/usr/far_planner/boundary_handler/data/");
    nh.param<std::string>(prefix + "boundary_file", ge_params_.bd_file_path, "boundary.ply");
    nh.param<std::string>(prefix + "traj_file", ge_params_.traj_file_path, "traj.txt");
    nh.param<std::string>(prefix + "graph_file", ge_params_.vgraph_path, "boundary_graph.vgh");
    nh.param<float>(prefix + "visual_scale_ratio", ge_params_.viz_scale_ratio, 1.0);
    nh.param<float>(prefix + "height_tolz", ge_params_.height_tolz, 1.0);
    ge_params_.vgraph_path    = folder_path + ge_params_.vgraph_path;
    ge_params_.bd_file_path   = folder_path + ge_params_.bd_file_path;
    ge_params_.traj_file_path = folder_path + ge_params_.traj_file_path;
    
}

void GraphExtractor::SetMarker(const VizColor& color, 
                             const std::string& ns,
                             const float scale,
                             const float alpha,  
                             Marker& scan_marker) 
{
    scan_marker.header.frame_id = ge_params_.frame_id;
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * ge_params_.viz_scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    this->SetColor(color, alpha, scan_marker);
}

void GraphExtractor::SetColor(const VizColor& color, 
                            const float alpha, 
                            Marker& scan_marker)
{
    std_msgs::ColorRGBA c;
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

void GraphExtractor::Run() {
    /* Main Running Function */
    ROS_INFO("Start reading boundary file ...");
    this->ReadBoundaryFile(boundary_ptr_, extracted_polys_);
    ROS_INFO("Boundary reading success, constructing V-Graph ...");
    this->ConstructVGraph(extracted_polys_, extracted_graph_);
    ROS_INFO("V-Graph constructed.");
    this->SaveVGraph(extracted_graph_);
    ROS_INFO("V-Graph saved, process terminated.");
}

void GraphExtractor::Loop() {
    this->VisualizeGraph(extracted_graph_);
    this->VisualizeFreePoint(free_p_);
}


void GraphExtractor::ConstructVGraph(const PolygonStack& polysIn, NodePtrStack& graphOut) {
    graphOut.clear();
    if (polysIn.empty()) return;
    for (const auto& poly_ptr : polysIn) {
        NavNodePtr temp_node_ptr = NULL;
        NodePtrStack node_stack;
        node_stack.clear();
        const int N = poly_ptr->vertices.size();
        for (std::size_t idx=0; idx<N; idx++) {
            this->CreateNavNode(poly_ptr->vertices[idx], temp_node_ptr);
            node_stack.push_back(temp_node_ptr);
        }
        // add connections to contour nodes
        for (int idx=0; idx<N; idx++) {
            const NavNodePtr cur_node_ptr = node_stack[idx];
            int ref_idx = this->Mod(idx-1, N);
            cur_node_ptr->surf_dirs.first = (node_stack[ref_idx]->position - cur_node_ptr->position).normalize_flat();
            ref_idx = this->Mod(idx+1, N);
            cur_node_ptr->surf_dirs.second = (node_stack[ref_idx]->position - cur_node_ptr->position).normalize_flat();
            // analysis convexity
            this->AnalysisConvexity(cur_node_ptr, poly_ptr);
            // connect boundary
            this->ConnectBoundary(node_stack[ref_idx], cur_node_ptr);
            graphOut.push_back(cur_node_ptr);
        }
    }
    const std::size_t GN = graphOut.size();
    // generate visbility connections
    for (std::size_t i=0; i<GN; i++) {
        for (std::size_t j=0; j<GN; j++) {
            if (i == j || j > i) continue;
            const NavNodePtr node_ptr1 = graphOut[i];
            const NavNodePtr node_ptr2 = graphOut[j];
            if (this->IsValidConnect(node_ptr1, node_ptr2)) {
                this->ConnectVEdge(node_ptr1, node_ptr2);
            }
        }
    }
}

bool GraphExtractor::IsValidConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return false;
    if (this->IsTypeInStack(node_ptr2->id, node_ptr1->contour_idxs)) return true;
    // check height tolerance
    if (abs(node_ptr1->position.z - node_ptr2->position.z) > ge_params_.height_tolz) return false;
    if (this->IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2) && this->IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2)) {
        return true;
    }
    return false;
}

bool GraphExtractor::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {

    const PointPair edge = this->ReprojectEdge(node_ptr1, node_ptr2, 0.05f);
    for (const auto& poly_ptr : extracted_polys_) {
        const std::size_t N = poly_ptr->N;
        for (std::size_t i=0; i<N; i++) {
            const PointPair line(poly_ptr->vertices[i], poly_ptr->vertices[this->Mod(i+1, N)]);
            if (this->IsEdgeCollideSegment(line, edge)) {
                return false;
            }
        }
    }
    return true;
}

PointPair GraphExtractor::ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist) {
    PointPair edgeOut;
    bool is_wall_UNUSE;
    if (node_ptr1->free_direct == NodeFreeDirect::CONVEX) {
        edgeOut.first  = node_ptr1->position - this->SurfTopoDirect(node_ptr1->surf_dirs, is_wall_UNUSE) * dist;
    } else {
        edgeOut.first  = node_ptr1->position + this->SurfTopoDirect(node_ptr1->surf_dirs, is_wall_UNUSE) * dist;
    }
    if (node_ptr2->free_direct == NodeFreeDirect::CONVEX) {
        edgeOut.second  = node_ptr2->position - this->SurfTopoDirect(node_ptr2->surf_dirs, is_wall_UNUSE) * dist;
    } else {
        edgeOut.second  = node_ptr2->position + this->SurfTopoDirect(node_ptr2->surf_dirs, is_wall_UNUSE) * dist;
    }
    return edgeOut;
}

bool GraphExtractor::IsEdgeCollideSegment(const PointPair& line, const PointPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);
    const cv::Point2f edge_p1(edge.first.x, edge.first.y);
    const cv::Point2f edge_p2(edge.second.x, edge.second.y);
    if (POLYOPS::doIntersect(start_p, end_p, edge_p1, edge_p2)) {
        return true;
    }
    return false;
}

bool GraphExtractor::IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    // check node1 -> node2
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!this->IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!this->IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
            return false;
        }
    }
    return true;
}

bool GraphExtractor::IsOutReducedDirs(const Point3D& diff_p, const PointPair& surf_dirs) {
    const Point3D norm_dir = diff_p.normalize_flat();
    Point3D temp_opposite_dir;
    // check first half range
    temp_opposite_dir = - surf_dirs.second;
    float in_res_thred = surf_dirs.first * temp_opposite_dir;
    if (norm_dir * surf_dirs.first > in_res_thred &&
        norm_dir * temp_opposite_dir > in_res_thred) {
        return true;
    }
    // check second half range
    temp_opposite_dir = - surf_dirs.first;
    in_res_thred = surf_dirs.second * temp_opposite_dir;
    if (norm_dir * surf_dirs.second > in_res_thred &&
        norm_dir * temp_opposite_dir > in_res_thred) {
        return true;
    }
    return false;
}

void GraphExtractor::AnalysisConvexity(const NavNodePtr& node_ptr, const PolygonPtr& poly_ptr) {
    bool is_wall = false;
    const Point3D topo_dir = this->SurfTopoDirect(node_ptr->surf_dirs, is_wall);
    if (is_wall) {
        node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        return;
    }
    const Point3D ev_p = node_ptr->position + topo_dir * 0.25f;
    if (this->IsConvexPoint(poly_ptr->vertices, ev_p, free_p_)) {
        node_ptr->free_direct = NodeFreeDirect::CONVEX;
    } else {
        node_ptr->free_direct = NodeFreeDirect::CONCAVE;
    }
}


void GraphExtractor::CreateNavNode(const Point3D& p, NavNodePtr& node_ptr) {
    node_ptr = std::make_shared<NavNode>();
    node_ptr->position = p;
    // assign global id
    node_ptr->id = id_, id_ ++;
    node_ptr->free_direct = NodeFreeDirect::UNKNOW;
    node_ptr->is_covered  = true;
    node_ptr->is_frontier = false;
    node_ptr->is_navpoint = false;
    node_ptr->is_boundary = true;
    // clear connection stacks
    node_ptr->connect_idxs.clear(), node_ptr->connect_nodes.clear();
    node_ptr->poly_idxs.clear(), node_ptr->poly_connects.clear();
    node_ptr->contour_idxs.clear(), node_ptr->contour_connects.clear();
    node_ptr->traj_idxs.clear(), node_ptr->traj_connects.clear();

}

void GraphExtractor::EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::Graph& graphOut) {
    graphOut.nodes.clear();
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        visibility_graph_msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.position    = ToGeoMsgP(node_ptr->position);
        msg_node.id          = node_ptr->id;
        msg_node.FreeType    = static_cast<int>(node_ptr->free_direct);
        msg_node.is_covered  = node_ptr->is_covered;
        msg_node.is_frontier = node_ptr->is_frontier;
        msg_node.is_navpoint = node_ptr->is_navpoint;
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
        graphOut.nodes.push_back(msg_node);
    }
    graphOut.size = graphOut.nodes.size();
}

void GraphExtractor::ReadBoundaryFile(const PointCloudPtr& bd_cloud, PolygonStack& polysOut) {
    /* Modified from waypointExample.cpp from https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/noetic/src/waypoint_example*/
    FILE* boundary_file = fopen(ge_params_.bd_file_path.c_str(), "r");
    if (boundary_file == NULL) {
        printf ("\nCannot read input boundary, exit.\n\n");
        exit(1);
    }
    char str[50];
    int val, pointNum;
    std::string strCur, strLast;
    while (strCur != "end_header") {
        val = fscanf(boundary_file, "%s", str);
        if (val != 1) {
            printf ("\nError reading boundary files, exit.\n\n");
            exit(1);
        }

        strLast = strCur;
        strCur = std::string(str);

        if (strCur == "vertex" && strLast == "element") {
            val = fscanf(boundary_file, "%d", &pointNum);
            if (val != 1) {
                printf ("\nError reading boundary files, exit.\n\n");
                exit(1);
            }
        }
    }

    bd_cloud->clear();
    PCLPoint point;
    int val1, val2, val3, val4;
    for (int i = 0; i < pointNum; i++) {
        val1 = fscanf(boundary_file, "%f", &point.x);
        val2 = fscanf(boundary_file, "%f", &point.y);
        val3 = fscanf(boundary_file, "%f", &point.z);
        val4 = fscanf(boundary_file, "%f", &point.intensity);

        if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
            printf ("\nError reading boundary values, exit.\n\n");
            exit(1);
        }
        bd_cloud->push_back(point);
    }
    // verify the polygon vertex size
    this->FilterPolyBoundary(bd_cloud, polysOut);
    fclose(boundary_file);
}

void GraphExtractor::ReadTrajFile(Point3D& free_p) {
    FILE* traj_file = fopen(ge_params_.traj_file_path.c_str(), "r");
    if (traj_file == NULL) {
        printf ("\nCannot read trajectory files, exit.\n\n");
        exit(1);
    }
    int valx, valy, valz;
    valx = fscanf(traj_file, "%f", &free_p.x);
    valy = fscanf(traj_file, "%f", &free_p.y);
    valz = fscanf(traj_file, "%f", &free_p.z);
    if (valx != 1 || valx != 1 || valx != 1) {
        printf ("\nError reading trajectory values, exit.\n\n");
        exit(1);
    }
    fclose(traj_file);
}

void GraphExtractor::FilterPolyBoundary(const PointCloudPtr& bd_cloud, PolygonStack& polysOut) {
    polysOut.clear();
    if (bd_cloud->empty()) return;
    int poly_id = (int)bd_cloud->points[0].intensity;
    PointStack vertices;
    vertices.clear();
    for (const auto& p : bd_cloud->points) {
        if ((int)p.intensity == poly_id) {
            vertices.push_back(Point3D(p));
        } else {
            if (vertices.size() > 2) { // at least 3 vertices to form a polygon
                PolygonPtr new_poly_ptr = NULL;
                this->CreatePolygon(vertices, new_poly_ptr, free_p_);
                polysOut.push_back(new_poly_ptr);
            }
            vertices.clear();
            vertices.push_back(Point3D(p));
            poly_id = (int)p.intensity;
        }
    }
    if (vertices.size() > 2) { // last polygon
        PolygonPtr new_poly_ptr = NULL;
        this->CreatePolygon(vertices, new_poly_ptr, free_p_);
        polysOut.push_back(new_poly_ptr);
    }
}

void GraphExtractor::SaveVGraph(const NodePtrStack& graphIn) {
    if (graphIn.empty()) return;
    const std::string file_path = ge_params_.vgraph_path;
    if (file_path == "") return;
    std::ofstream graph_file;
    graph_file.open(file_path);
    // Lambda function
    auto OutputPoint3D = [&](const Point3D& p) {
        graph_file << std::to_string(p.x) << " ";
        graph_file << std::to_string(p.y) << " ";
        graph_file << std::to_string(p.z) << " ";
    };
    for (const auto& node_ptr : graphIn) {
        graph_file << std::to_string(node_ptr->id) << " ";
        graph_file << std::to_string(static_cast<int>(node_ptr->free_direct)) << " ";
        OutputPoint3D(node_ptr->position);
        OutputPoint3D(node_ptr->surf_dirs.first);
        OutputPoint3D(node_ptr->surf_dirs.second);
        graph_file << std::to_string(static_cast<int>(node_ptr->is_covered))  << " ";
        graph_file << std::to_string(static_cast<int>(node_ptr->is_frontier)) << " ";
        graph_file << std::to_string(static_cast<int>(node_ptr->is_navpoint)) << " ";
        graph_file << std::to_string(static_cast<int>(node_ptr->is_boundary)) << " ";
        for (const auto& cidx : node_ptr->connect_idxs) {
            graph_file << std::to_string(cidx) << " ";
        }
        graph_file << "|" << " ";
        for (const auto& cidx : node_ptr->poly_idxs) {
            graph_file << std::to_string(cidx) << " ";
        } 
        graph_file << "|" << " ";
        for (const auto& cidx : node_ptr->contour_idxs) {
            graph_file << std::to_string(cidx) << " ";
        }
        graph_file << "|" << " ";
        for (const auto& cidx : node_ptr->traj_idxs) {
            graph_file << std::to_string(cidx) << " ";
        }
        graph_file << "\n";
    }
    graph_file.close();
}

void GraphExtractor::VisualizeFreePoint(const Point3D& free_p) {
    Marker node_marker;
    node_marker.type = Marker::SPHERE;
    this->SetMarker(VizColor::GREEN, "free_position", 1.25f,  0.75f, node_marker);
    std::size_t idx = 0;
    node_marker.pose.position.x = free_p.x;
    node_marker.pose.position.y = free_p.y;
    node_marker.pose.position.z = free_p.z;
    viz_node_pub_.publish(node_marker);
}

void GraphExtractor::VisualizeGraph(const NodePtrStack& graphIn) {
    MarkerArray graph_marker_array;
    Marker nav_node_marker, covered_node_marker, internav_node_marker, frontier_node_marker, edge_marker, poly_edge_marker, 
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
        geometry_msgs::Point p1, p2;
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
        geometry_msgs::Point p1, p2, p3;
        p1 = ToGeoMsgP(node_ptr->position);
        Point3D end_p;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            end_p = node_ptr->position + node_ptr->surf_dirs.first * ge_params_.viz_scale_ratio;
            p2 = ToGeoMsgP(end_p);
            corner_surf_marker.points.push_back(p1);
            corner_surf_marker.points.push_back(p2);
            corner_helper_marker.points.push_back(p2);
            end_p = node_ptr->position + node_ptr->surf_dirs.second * ge_params_.viz_scale_ratio;
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
        const geometry_msgs::Point cpoint = ToGeoMsgP(nav_node_ptr->position);
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
    graph_marker_array.markers.push_back(nav_node_marker);
    graph_marker_array.markers.push_back(covered_node_marker);
    graph_marker_array.markers.push_back(internav_node_marker);
    graph_marker_array.markers.push_back(frontier_node_marker);
    graph_marker_array.markers.push_back(edge_marker);
    graph_marker_array.markers.push_back(poly_edge_marker);
    graph_marker_array.markers.push_back(boundary_edge_marker);
    graph_marker_array.markers.push_back(free_edge_marker);
    graph_marker_array.markers.push_back(contour_edge_marker);
    graph_marker_array.markers.push_back(traj_edge_marker);
    graph_marker_array.markers.push_back(corner_surf_marker);
    graph_marker_array.markers.push_back(corner_helper_marker);
    graph_viz_pub_.publish(graph_marker_array);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "boundary_graph_node");
  GraphExtractor ge_node;
  ge_node.Init();
  ge_node.Run();
  ros::Rate loop_rate(1.0);
    while (ros::ok()) {
        ros::spinOnce();
        ge_node.Loop();
        loop_rate.sleep();
    }
}
