// Copyright (c) 2021-2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include "point_struct.h"
#include <boost/functional/hash.hpp>

enum NodeType {
    NOT_DEFINED = 0,
    GROUND      = 1,
    AIR         = 2
};

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3
};

typedef std::pair<Point3D, Point3D> PointPair;

namespace LiDARMODEL {
    /* array resolution: 1 degree */
    static const int kHorizontalFOV = 360;  
    static const int kVerticalFOV = 31; 
    static const float kAngleResX = 0.2;
    static const float kAngleResY = 2.0;    
} // NODEPARAMS

struct Polygon
{
  Polygon() = default;
  std::size_t N = 0;
  std::vector<Point3D> vertices;
  bool is_robot_inside = false;
  bool is_pillar = false;
  float perimeter = 0.0f;
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct CTNode
{
    CTNode() = default;
    Point3D position;
    bool is_global_match = false;
    bool is_contour_necessary = false;
    bool is_ground_associate = false;
    std::size_t nav_node_id = 0;
    NodeFreeDirect free_direct = NodeFreeDirect::UNKNOW;

    PointPair surf_dirs;
    PolygonPtr poly_ptr;
    std::shared_ptr<CTNode> front;
    std::shared_ptr<CTNode> back;

};

typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;

struct NavNode
{
    NavNode() = default;
    std::size_t id = 0;
    Point3D position;
    PointPair surf_dirs;
    std::deque<Point3D> pos_filter_vec;
    std::deque<PointPair> surf_dirs_vec;
    CTNodePtr ctnode;
    bool is_active = false;
    bool is_block_frontier = false;
    bool is_contour_match = false;
    bool is_odom = false;
    bool is_goal = false;
    bool is_near_nodes = false;
    bool is_wide_near = false;
    bool is_merged = false;
    bool is_covered = false;
    bool is_frontier = false;
    bool is_finalized = false;
    bool is_navpoint = false;
    bool is_boundary = false;
    int  clear_dumper_count = 0;
    std::deque<int> frontier_votes;
    std::unordered_set<std::size_t> invalid_boundary;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;
    std::vector<std::shared_ptr<NavNode>> poly_connects;
    std::vector<std::shared_ptr<NavNode>> contour_connects;
    std::unordered_map<std::size_t, std::deque<int>> contour_votes;
    std::unordered_map<std::size_t, std::deque<int>> edge_votes;
    std::vector<std::shared_ptr<NavNode>> potential_contours;
    std::vector<std::shared_ptr<NavNode>> potential_edges;
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;
    std::unordered_map<std::size_t, std::size_t> trajectory_votes;
    std::unordered_map<std::size_t, std::size_t> terrain_votes;
    NodeType node_type = NodeType::NOT_DEFINED;
    NodeFreeDirect free_direct = NodeFreeDirect::UNKNOW;
    // planner members
    bool is_block_to_goal = false;
    bool is_traversable = false;
    bool is_free_traversable = false;
    float gscore = 0.0f, fgscore = 0.0f;
    std::shared_ptr<NavNode> parent;
    std::shared_ptr<NavNode> free_parent;
    
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavNodePtr> NavEdge;

struct nodeptr_equal
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->id == n2->id;
  }
};

struct navedge_hash
{
  std::size_t operator() (const NavEdge& nav_edge) const
  {
    return boost::hash<std::pair<std::size_t, std::size_t>>()({nav_edge.first->id, nav_edge.second->id});
  }
};

struct nodeptr_hash
{
  std::size_t operator() (const NavNodePtr& n_ptr) const
  {
    return std::hash<std::size_t>()(n_ptr->id);
  }
};

struct nodeptr_icomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->position.intensity < n2->position.intensity;
  }
};

#endif
