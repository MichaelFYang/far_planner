/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/dynamic_graph.h"

/***************************************************************************************/

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
    id_tracker_     = 1;
    last_connect_pos_ = Point3D(0,0,0);
    /* Initialize Terrian Planner */
    tp_params_.world_frame  = FARUtil::worldFrameId;
    tp_params_.voxel_size   = FARUtil::kLeafSize;
    tp_params_.radius       = FARUtil::kNearDist * 2.0f;
    tp_params_.inflate_size = FARUtil::kObsInflate;
    terrain_planner_.Init(nh, tp_params_);
}

void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
    robot_pos_ = robot_pos;
    terrain_planner_.SetLocalTerrainObsCloud(FARUtil::local_terrain_obs_);
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);
        this->AddNodeToGraph(odom_node_ptr_);
        if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
    } else {
        this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
    }
    FARUtil::odom_pos = odom_node_ptr_->position;
    terrain_planner_.VisualPaths();
}

bool DynamicGraph::IsInterNavpointNecessary() {
    if (cur_internav_ptr_ == NULL ) { // create nearest nav point
        last_connect_pos_ = FARUtil::free_odom_p;
        return true;
    }
    const auto it = odom_node_ptr_->edge_votes.find(cur_internav_ptr_->id);
    if (is_bridge_internav_ || it == odom_node_ptr_->edge_votes.end() || !this->IsInternavInRange(cur_internav_ptr_)) {
        float min_dist = FARUtil::kINF;
        for (const auto& internav_ptr : internav_near_nodes_) {
            const float cur_dist = (internav_ptr->position - last_connect_pos_).norm();
            if (cur_dist < min_dist) min_dist = cur_dist;
        }
        if (min_dist > FARUtil::kNavClearDist) return true;
    } 
    if ((FARUtil::free_odom_p - last_connect_pos_).norm() > FARUtil::kNearDist || 
        (it != odom_node_ptr_->edge_votes.end() && it->second.back() == 1)) 
    {
        last_connect_pos_ = FARUtil::free_odom_p;
    }
    return false;
}

bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    if (new_ctnodes.empty()) return false;
    NavNodePtr new_node_ptr = NULL;
    new_nodes_.clear();
    if (this->IsInterNavpointNecessary()) { // check wheter or not need inter navigation points
        if (FARUtil::IsDebug) ROS_INFO("DG: One trajectory node has been created.");
        this->CreateNavNodeFromPoint(last_connect_pos_, new_node_ptr, false, true);
        new_nodes_.push_back(new_node_ptr);
        last_connect_pos_ = FARUtil::free_odom_p;
        if (is_bridge_internav_) is_bridge_internav_ = false;
    }
    for (const auto& ctnode_ptr : new_ctnodes) {
        bool is_near_new = false;
        if (this->IsAValidNewNode(ctnode_ptr, is_near_new)) {
            this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
            if (!is_near_new) {
                new_node_ptr->is_block_frontier = true;
            }
            new_nodes_.push_back(new_node_ptr);
        }
    }
    if (new_nodes_.empty()) return false;
    else return true;
}

void DynamicGraph::UpdateNavGraph(const NodePtrStack& new_nodes,
                                  const bool& is_freeze_vgraph,
                                  NodePtrStack& clear_node) 
{
    // clear false positive node detection
    clear_node.clear();
    if (!is_freeze_vgraph) {
        for (const auto& node_ptr : extend_match_nodes_) {
            if (FARUtil::IsStaticNode(node_ptr) || node_ptr == cur_internav_ptr_) continue;
            if (!this->ReEvaluateCorner(node_ptr)) {
                if (this->SetNodeToClear(node_ptr)) {
                    clear_node.push_back(node_ptr);
                }
            } else {
                this->ReduceDumperCounter(node_ptr);
            }
        }
        // re-evaluate trajectory edge using terrain planner
        if (!FARUtil::IsStaticEnv && cur_internav_ptr_ != NULL) {
            NodePtrStack internav_check_nodes = surround_internav_nodes_;
            if (!FARUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
                internav_check_nodes.push_back(cur_internav_ptr_);
            }
            for (const auto& sur_internav_ptr : internav_check_nodes) {
                const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
                for (const auto& tnode_ptr : copy_traj_connects) {
                if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                        this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    } else {
                        this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    }
                }   
            }     
        }
    }
    // clear merged nodes in stacks
    this->ClearMergedNodesInGraph();
    // add matched margin nodes into near and wide near nodes
    this->UpdateNearNodesWithMatchedMarginNodes(margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);
    // check-add connections to odom node with wider near nodes
    NodePtrStack codom_check_list = wide_near_nodes_;
    codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end()); // add new nodes to check list
    for (const auto& conode_ptr : codom_check_list) {
        if (conode_ptr->is_odom) continue;
        if (this->IsValidConnect(odom_node_ptr_, conode_ptr, false)) {
            this->AddPolyEdge(odom_node_ptr_, conode_ptr), this->AddEdge(odom_node_ptr_, conode_ptr);
        } else {
            this->ErasePolyEdge(odom_node_ptr_, conode_ptr), this->EraseEdge(conode_ptr, odom_node_ptr_);
        }
    }
    if (!is_freeze_vgraph) {
        // Adding new nodes to near nodes stack
        for (const auto& new_node_ptr : new_nodes) {
            this->AddNodeToGraph(new_node_ptr);
            new_node_ptr->is_near_nodes = true;
            near_nav_nodes_.push_back(new_node_ptr);
            if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
            if (new_node_ptr->ctnode != NULL) {
                ContourGraph::MatchCTNodeWithNavNode(new_node_ptr->ctnode, new_node_ptr);
            }
        }
        // connect outrange contour nodes
        for (const auto& out_node_ptr : out_contour_nodes_) {
            const NavNodePtr matched_node = ContourGraph::MatchOutrangeNodeWithCTNode(out_node_ptr, near_nav_nodes_);
            const auto it = out_contour_nodes_map_.find(out_node_ptr);
            if (matched_node != NULL) {
                this->RecordContourVote(out_node_ptr, matched_node);
                it->second.second.insert(matched_node);

            }
            for (const auto& reached_node_ptr : it->second.second) {
                if (reached_node_ptr != matched_node) {
                    this->DeleteContourVote(out_node_ptr, reached_node_ptr);
                }
            }
        }
        // reconnect between near nodes
        NodePtrStack outside_break_nodes;
        for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
            const NavNodePtr nav_ptr1 = near_nav_nodes_[i];
            if (nav_ptr1->is_odom) continue;
            // re-evaluate nodes which are not in near
            const NodePtrStack copy_connect_nodes = nav_ptr1->connect_nodes;
            for (const auto& cnode : copy_connect_nodes) {
                if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) || FARUtil::IsTypeInStack(cnode, nav_ptr1->contour_connects)) continue;
                if (this->IsValidConnect(nav_ptr1, cnode, false)) {
                    this->AddPolyEdge(nav_ptr1, cnode), this->AddEdge(nav_ptr1, cnode);
                } else {
                    this->ErasePolyEdge(nav_ptr1, cnode) ,this->EraseEdge(nav_ptr1, cnode);
                    outside_break_nodes.push_back(cnode);
                } 
            }
            for (std::size_t j=0; j<near_nav_nodes_.size(); j++) {
                const NavNodePtr nav_ptr2 = near_nav_nodes_[j];
                if (i == j || j > i || nav_ptr2->is_odom) continue;
                if (this->IsValidConnect(nav_ptr1, nav_ptr2, true)) {
                    this->AddPolyEdge(nav_ptr1, nav_ptr2), this->AddEdge(nav_ptr1, nav_ptr2);
                } else {
                    this->ErasePolyEdge(nav_ptr1, nav_ptr2), this->EraseEdge(nav_ptr1, nav_ptr2);
                }
            }
            for (const auto& oc_node_ptr : out_contour_nodes_) {
                if (!oc_node_ptr->is_contour_match || !nav_ptr1->is_contour_match) continue;
                if (ContourGraph::IsNavNodesConnectFromContour(nav_ptr1, oc_node_ptr)) {
                    this->RecordContourVote(nav_ptr1, oc_node_ptr);
                } else {
                    this->DeleteContourVote(nav_ptr1, oc_node_ptr);
                }
            }
            this->TopTwoContourConnector(nav_ptr1);
        }
        // update out range break nodes connects
        for (const auto& node_ptr : near_nav_nodes_) {
            for (const auto& ob_node_ptr : outside_break_nodes) {
                if (this->IsValidConnect(node_ptr, ob_node_ptr, false)) {
                    this->AddPolyEdge(node_ptr, ob_node_ptr), this->AddEdge(node_ptr, ob_node_ptr);
                } else {
                    this->ErasePolyEdge(node_ptr, ob_node_ptr), this->EraseEdge(node_ptr, ob_node_ptr);
                }
            }
        }
        // Analysisig frontier nodes
        for (const auto& node_ptr : near_nav_nodes_) {
            if (this->IsNodeFullyCovered(node_ptr)) {
                node_ptr->is_covered = true;
            } else {
                node_ptr->is_covered = false;
            }
            if (this->IsFrontierNode(node_ptr)) {
                node_ptr->is_frontier = true;
            } else {
                node_ptr->is_frontier = false;
            }
        }
    }
}

bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2,
                                  const bool& is_check_contour) 
{
    const float dist = (node_ptr1->position - node_ptr2->position).norm();
    if (dist < FARUtil::kEpsilon) return true;
    if ((node_ptr1->is_odom || node_ptr2->is_odom) && (node_ptr1->is_navpoint || node_ptr2->is_navpoint)) {
        if (dist < FARUtil::kNavClearDist) return true; 
    } 
    /* check contour connection from node1 to node2 */
    if (is_check_contour) {
        if (this->IsBoundaryConnect(node_ptr1, node_ptr2) || (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, true))) {
            this->RecordContourVote(node_ptr1, node_ptr2);
        } else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            this->DeleteContourVote(node_ptr1, node_ptr2);
        }
    }
    bool is_connect = false;
    /* check polygon connections */
    const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size / 3.0f) : dg_params_.votes_size;
    if (IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2) && ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, false)) {
        if (this->IsPolyMatchedForConnect(node_ptr1, node_ptr2)) {
            RecordPolygonVote(node_ptr1, node_ptr2, vote_queue_size);
        }
    } else {
        DeletePolygonVote(node_ptr1, node_ptr2, vote_queue_size);
    }
    if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
        if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) is_connect = true;
    } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
        node_ptr1->edge_votes.erase(node_ptr2->id);
        node_ptr2->edge_votes.erase(node_ptr1->id);
        // clear potential connections
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
    }
    /* check if exsiting trajectory connection exist */
    if (!is_connect) {
        if (FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
        if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
            if (node_ptr1->is_odom && FARUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
                if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr2->position, node_ptr1->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }   
            } else if (node_ptr2->is_odom && FARUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
                if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr1->position, node_ptr2->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }
            }
        }
    }
    /* check for additional contour connection through tight area from current robot position */
    if (!is_connect && (node_ptr1->is_odom || node_ptr2->is_odom) && IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2)) {
        if (node_ptr1->is_odom && !node_ptr2->contour_connects.empty()) {
            for (const auto& ctnode_ptr : node_ptr2->contour_connects) {
                if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr2->position, node_ptr1->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        } else if (node_ptr2->is_odom && !node_ptr1->contour_connects.empty()) {
            for (const auto& ctnode_ptr : node_ptr1->contour_connects) {
                if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr1->position, node_ptr2->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        }
    }
    return is_connect;
}

bool DynamicGraph::IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour) {
        if (!node_ptr1->is_active || !node_ptr2->is_active) return true;
        Point3D mid_p = (node_ptr1->position + node_ptr2->position) / 2.0f;
        const Point3D diff_p = node_ptr2->position - node_ptr1->position;
        if (diff_p.norm() > FARUtil::kMatchDist && abs(diff_p.z) / std::hypotf(diff_p.x, diff_p.y) > 1) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false; // slope is too steep > 45 degree
        } 
        if (is_contour && node_ptr1->contour_votes.find(node_ptr2->id) != node_ptr1->contour_votes.end()) { // recorded contour terrain connection
            return true;
        }
        bool is_match;
        float minH, maxH;
        const float avg_h = MapHandler::NearestHeightOfRadius(mid_p, FARUtil::kMatchDist, minH, maxH, is_match);
        if (!is_match && (is_contour || !node_ptr1->is_frontier || !node_ptr2->is_frontier)) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false;
        } 
        if (is_match && (maxH - minH > FARUtil::kMarginHeight || abs(minH + FARUtil::vehicle_height - mid_p.z) > FARUtil::kTolerZ / 2.0f)) {
            if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
            return false;
        }
        if (!is_contour) {
            if (is_match) RecordVaildTerrainConnect(node_ptr1, node_ptr2);
            const auto it = node_ptr1->terrain_votes.find(node_ptr2->id);
            if (it != node_ptr1->terrain_votes.end() && it->second > dg_params_.finalize_thred) {
                return false;
            }
        }
        return true;
    }

bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    if (FARUtil::IsFreeNavNode(node_ptr) || node_ptr->is_covered) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    for (const auto& near_optr : check_odom_list) {
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < FARUtil::kMatchDist) return true;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            // TODO: concave nodes will not be marked as covered based on current implementation
            const auto it = near_optr->edge_votes.find(node_ptr->id);
            if (it != near_optr->edge_votes.end() && FARUtil::IsVoteTrue(it->second)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicGraph::IsFrontierNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_contour_match) {
        if (node_ptr->is_block_frontier || node_ptr->is_covered || node_ptr->free_direct != NodeFreeDirect::CONVEX ||
            node_ptr->ctnode->poly_ptr->perimeter < dg_params_.frontier_perimeter_thred) 
        {
            node_ptr->frontier_votes.push_back(0); // non convex frontier or too small
        } else {
            node_ptr->frontier_votes.push_back(1); // convex frontier
        }
    } else if (!FARUtil::IsPointInMarginRange(node_ptr->position)) { // if not in margin range, the node won't be deleted
        node_ptr->frontier_votes.push_back(0); // non convex frontier
    }
    if (node_ptr->frontier_votes.size() > dg_params_.finalize_thred) {
        node_ptr->frontier_votes.pop_front();
    }
    bool is_frontier = FARUtil::IsVoteTrue(node_ptr->frontier_votes);
    if (!node_ptr->is_frontier && is_frontier && node_ptr->frontier_votes.size() == dg_params_.finalize_thred) {
        if (!FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
            is_frontier = false;
        }
    }
    return is_frontier;
}

bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                              const NavNodePtr& node_ptr_to)
{
    // TODO: check for connection loss
    if (node_ptr_from->is_odom || node_ptr_to->is_odom) return false;
    if (FARUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) { // release for contour connection
        return false;
    }
    // check from to to node connection
    if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
        return true;
    }
    if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
        return true;
    }
    return false;
}

bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2) 
{
    // check for odom -> frontier connections
    if ((node_ptr1->is_odom && node_ptr2->is_frontier) || (node_ptr2->is_odom && node_ptr1->is_frontier)) return true;
    // check node1 -> node2
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!FARUtil::IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!FARUtil::IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsInContourDirConstraint(const NavNodePtr& node_ptr1,
                                            const NavNodePtr& node_ptr2) 
{
    if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return false;
    // check node1 -> node2
    if (node_ptr1->is_finalized && node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_1to2 = node_ptr2->position - node_ptr1->position;
        if (!FARUtil::IsInContourDirPairs(diff_1to2, node_ptr1->surf_dirs)) {
            if (node_ptr1->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr1);
            } 
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->is_finalized && node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_2to1 = node_ptr1->position - node_ptr2->position;
        if (!FARUtil::IsInContourDirPairs(diff_2to1, node_ptr2->surf_dirs)) {
            if (node_ptr2->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr2);
            }
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    bool is_nav_connect = false;
    bool is_cover_connect = false;
    if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
    if (node_ptr_from->is_covered && node_ptr_to->is_covered) is_cover_connect = true;
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && !cnode->is_covered) continue;
        if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || ref_diff.norm() < FARUtil::kEpsilon) continue;
        ref_dir = ref_diff.normalize();
        if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
            return true;
        }
    }
    return false;
}

bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr,
                                      const Point3D& new_pos) 
{
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        this->InitNodePosition(node_ptr, new_pos);
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    node_ptr->pos_filter_vec.push_back(new_pos);
    if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
        node_ptr->pos_filter_vec.pop_front();
    }
    // calculate mean nav node position using RANSACS
    std::size_t inlier_size = 0;
    Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
    if (node_ptr->pos_filter_vec.size() > 1) mean_p.z = node_ptr->position.z; // keep z value with terrain updates
    node_ptr->position = mean_p;
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
}

void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    node_ptr->pos_filter_vec.clear();
    node_ptr->position = new_pos;
    node_ptr->pos_filter_vec.push_back(new_pos);
}

bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs)
{
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    FARUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
    node_ptr->surf_dirs_vec.push_back(cur_dirs);
    if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
        node_ptr->surf_dirs_vec.pop_front();
    }
    // calculate mean surface corner direction using RANSACS
    std::size_t inlier_size = 0;
    const PointPair mean_dir = FARUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
    if (mean_dir.first == Point3D(0,0,-1) || mean_dir.second == Point3D(0,0,-1)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
    } else {
        node_ptr->surf_dirs = mean_dir;
        this->ReEvaluateConvexity(node_ptr);
    }
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;       
}

void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
    if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar) return;
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs, is_wall);
    if (!is_wall) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
        if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr, ev_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}

void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr) {
    std::vector<int> votesc;
    for (const auto& vote : node_ptr->contour_votes) {
        if (FARUtil::IsVoteTrue(vote.second, false)) {
            votesc.push_back(std::accumulate(vote.second.begin(), vote.second.end(), 0));
        }
    }
    std::sort(votesc.begin(), votesc.end(), std::greater<int>());
    for (const auto& cnode_ptr : node_ptr->potential_contours) {
        const auto it = node_ptr->contour_votes.find(cnode_ptr->id);
        // DEBUG
        if (it == node_ptr->contour_votes.end()) ROS_ERROR("DG: contour potential node matching error");
        const int itc = std::accumulate(it->second.begin(), it->second.end(), 0);
        if (FARUtil::VoteRankInVotes(itc, votesc) < 2 && FARUtil::IsVoteTrue(it->second, false)) {
            DynamicGraph::AddContourConnect(node_ptr, cnode_ptr);
            this->AddEdge(node_ptr, cnode_ptr);
        } else if (DynamicGraph::DeleteContourConnect(node_ptr, cnode_ptr) && !FARUtil::IsTypeInStack(cnode_ptr, node_ptr->poly_connects)) {
            this->EraseEdge(node_ptr, cnode_ptr);
        }
    }
}

void DynamicGraph::RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init contour connection votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
        }
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::RecordPolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size, 
                                     const bool& is_reset) 
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        // init polygon edge votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
        }
        if (is_reset) it1->second.clear(), it2->second.clear();
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > queue_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::FillPolygonEdgeConnect(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2,
                                        const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        std::deque<int> vote_queue1(queue_size, 1);
        std::deque<int> vote_queue2(queue_size, 1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) 
        {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
        // Add connections
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) 
        {
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::FillContourConnect(const NavNodePtr& node_ptr1,
                                    const NavNodePtr& node_ptr2,
                                    const int& queue_size)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    std::deque<int> vote_queue1(queue_size, 1);
    std::deque<int> vote_queue2(queue_size, 1);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init polygon edge votes
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && 
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) 
        {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
        // Add contours
        DynamicGraph::AddContourConnect(node_ptr1, node_ptr2);
    }
}

void DynamicGraph::FillTrajConnect(const NavNodePtr& node_ptr1,
                                 const NavNodePtr& node_ptr2)
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end()) {
        node_ptr1->trajectory_votes.insert({node_ptr2->id, 0});
        node_ptr2->trajectory_votes.insert({node_ptr1->id, 0});
        // Add connection
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->trajectory_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) 
        {   
            node_ptr1->trajectory_connects.push_back(node_ptr2);
            node_ptr2->trajectory_connects.push_back(node_ptr1);
        }
    }
}

void DynamicGraph::DeletePolygonVote(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size,
                                     const bool& is_reset) 
{
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
    if (is_reset) it1->second.clear(), it2->second.clear();
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > queue_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

/* Delete Contour edge for given two navigation nodes */
void DynamicGraph::DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return; // no connection (not counter init) in the first place 
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > dg_params_.votes_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

bool DynamicGraph::IsActivateNavNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_active) return true;
    if (FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
        node_ptr->is_active = true;
        return true;
    }
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        const bool is_nearby = (node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist ? true : false;
        if (is_nearby) {
            node_ptr->is_active = true;
            return true;
        }
        if (FARUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
            node_ptr->is_active = true;
            return true;
        }
        bool is_connects_activate = true;
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!cnode_ptr->is_active) {
                is_connects_activate = false;
                break;
            }
        }
        if ((is_connects_activate && !node_ptr->connect_nodes.empty())) {
            node_ptr->is_active = true;
            return true;
        }
    }
    return false;
}

void DynamicGraph::UpdateGlobalNearNodes() {
    /* update nearby navigation nodes stack --> near_nav_nodes_ */
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), extend_match_nodes_.clear();
    margin_near_nodes_.clear(); internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    for (const auto& node_ptr : globalGraphNodes_) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near  = false;
        if (FARUtil::IsNodeInExtendMatchRange(node_ptr) && (!node_ptr->is_active || MapHandler::IsNavPointOnTerrainNeighbor(node_ptr->position, true))) {
            if (FARUtil::IsOutsideGoal(node_ptr)) continue;
            if (this->IsActivateNavNode(node_ptr) || node_ptr->is_boundary) extend_match_nodes_.push_back(node_ptr);
            if (FARUtil::IsNodeInLocalRange(node_ptr) && IsPointOnTerrain(node_ptr->position)) {
                wide_near_nodes_.push_back(node_ptr);
                node_ptr->is_wide_near = true;
                if (node_ptr->is_active || node_ptr->is_boundary) {
                    near_nav_nodes_.push_back(node_ptr);
                    node_ptr->is_near_nodes = true;
                    if (node_ptr->is_navpoint) {
                        node_ptr->position.intensity = node_ptr->fgscore;
                        internav_near_nodes_.push_back(node_ptr);
                        if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kLocalPlanRange / 2.0f) {
                            surround_internav_nodes_.push_back(node_ptr);
                        }
                    }
                }
            } else if (node_ptr->is_active || node_ptr->is_boundary) {
                margin_near_nodes_.push_back(node_ptr);
            }
        }
    }
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { // add additional odom connections to wide near stack
        if (FARUtil::IsOutsideGoal(cnode_ptr)) continue;
        if (!cnode_ptr->is_wide_near) {
            wide_near_nodes_.push_back(cnode_ptr);
            cnode_ptr->is_wide_near = true;
        }
        for (const auto& c2node_ptr : cnode_ptr->connect_nodes) {
            if (!c2node_ptr->is_wide_near && !FARUtil::IsOutsideGoal(c2node_ptr)) {
                wide_near_nodes_.push_back(c2node_ptr);
                c2node_ptr->is_wide_near = true;
            }
        }
    }
    if (!internav_near_nodes_.empty()) { // find the nearest inter_nav node that connect to odom
        std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp());
        for (std::size_t i=0; i<internav_near_nodes_.size(); i++) {
            const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];
            if (FARUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) && this->IsInternavInRange(temp_internav_ptr)) {
                if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ || (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < FARUtil::kNearDist ||
                    FARUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) 
                {   
                    this->UpdateCurInterNavNode(temp_internav_ptr);  
                } else {
                    is_bridge_internav_ = true;
                }
                break;
            }
        }
    }
}

bool DynamicGraph::ReEvaluateCorner(const NavNodePtr node_ptr) {
    if (node_ptr->is_boundary) return true;
    if (node_ptr->is_navpoint) {
        if (FARUtil::IsTypeInStack(node_ptr, surround_internav_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
            return false;
        }
        return true;
    }
    const bool is_near_new = FARUtil::IsPointNearNewPoints(node_ptr->position, false);
    if (is_near_new) { // if nearby env changes;
        this->ResetNodeFilters(node_ptr);
        if (!node_ptr->is_contour_match) this->ResetNodeConnectVotes(node_ptr);
    }
    if (!node_ptr->is_contour_match) {
        if (FARUtil::IsPointInMarginRange(node_ptr->position) || is_near_new) return false;
        return true;
    }
    if (node_ptr->is_finalized) return true;

    bool is_pos_cov  = false;
    bool is_dirs_cov = false;
    if (node_ptr->is_contour_match) {
        is_pos_cov  = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
        is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
        if (FARUtil::IsDebug) ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
    }
    if (is_pos_cov && is_dirs_cov) node_ptr->is_finalized = true;

    return true;
}

bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
    PointStack terrain_path;
    if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
        return true;
    }
    return false;
}
