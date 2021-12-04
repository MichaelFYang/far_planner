/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/dynamic_graph.h"

/***************************************************************************************/

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    near_nav_nodes_.clear();
    wide_near_nodes_.clear();
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
    ALIGN_ANGLE_COS = cos(M_PI - FARUtil::kAcceptAlign);
    TRAJ_DIST       = dg_params_.sensor_range / dg_params_.traj_interval_ratio;
    MARGIN_DIST     = dg_params_.sensor_range - dg_params_.margin_dist;
    id_tracker_     = 0;
    last_connect_pos_ = Point3D(0,0,0);
    /* Initialize Terrian Planner */
    tp_params_.world_frame  = FARUtil::worldFrameId;
    tp_params_.local_range  = TRAJ_DIST;
    tp_params_.voxel_size   = FARUtil::kLeafSize;
    tp_params_.radius       = FARUtil::kNearDist * 2.0f;
    tp_params_.inflate_size = dg_params_.terrain_inflate;
    terrain_planner_.Init(nh, tp_params_);
}

void DynamicGraph::UpdateOdom(const Point3D& robot_pos) {
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
    this->UpdateGlobalNearNodes();
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
        if (this->IsAValidNewNode(ctnode_ptr)) {
            this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
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
        for (const auto& node_ptr : near_nav_nodes_) {
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
    // check-add connections to odom node with wider near nodes
    NodePtrStack codom_check_list = wide_near_nodes_;
    codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end());
    for (const auto& conode_ptr : codom_check_list) {
        if (conode_ptr->is_odom || conode_ptr->is_merged) continue;
        if (this->IsValidConnect(odom_node_ptr_, conode_ptr, false)) {
            this->AddEdge(odom_node_ptr_, conode_ptr);
            if (conode_ptr->is_contour_match) {
                conode_ptr->ctnode->poly_ptr->is_visiable = true;
            }
        } else {
            this->EraseEdge(conode_ptr, odom_node_ptr_);
        }
    }
    if (!is_freeze_vgraph) {
        // Adding new nodes to near nodes stack
        for (const auto& new_node_ptr : new_nodes) {
            this->AddNodeToGraph(new_node_ptr);
            new_node_ptr->is_near_nodes = true;
            near_nav_nodes_.push_back(new_node_ptr);
            if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
            if (new_node_ptr->ctnode != NULL) new_node_ptr->ctnode->is_global_match = true;
        }
        // Adjust Z axis coordinate if for Ground only navigation
        for (const auto& node_ptr : near_nav_nodes_) {
            if (!FARUtil::IsFreeNavNode(node_ptr) && this->IsVisibleNode(node_ptr)) {
                this->AdjustNavNodeZAxis(node_ptr);
            }
        }
        // reconnect between near nodes
        NodePtrStack outside_break_nodes;
        for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
            if (near_nav_nodes_[i]->is_merged || near_nav_nodes_[i]->is_odom) continue;
            // re-evaluate nodes which are not in near
            const NodePtrStack copy_connect_nodes = near_nav_nodes_[i]->connect_nodes;
            for (const auto& cnode : copy_connect_nodes) {
                if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) || FARUtil::IsTypeInStack(cnode, near_nav_nodes_[i]->contour_connects)) continue;
                //TODO: Delete outrange contour conncetions that cross layer
                if (this->IsValidConnect(near_nav_nodes_[i], cnode, false)) {
                    this->AddEdge(near_nav_nodes_[i], cnode);
                } else {
                    this->EraseEdge(near_nav_nodes_[i], cnode);
                    outside_break_nodes.push_back(cnode);
                } 
            }
            for (std::size_t j=0; j<near_nav_nodes_.size(); j++) {
                if (i == j || j > i || near_nav_nodes_[j]->is_merged || near_nav_nodes_[j]->is_odom) continue;
                if (this->IsValidConnect(near_nav_nodes_[i], near_nav_nodes_[j], true)) {
                    this->AddEdge(near_nav_nodes_[i], near_nav_nodes_[j]);
                } else {
                    this->EraseEdge(near_nav_nodes_[i], near_nav_nodes_[j]);
                }
            }
        }
        // update out range break nodes connects
        for (const auto& node_ptr : near_nav_nodes_) {
            for (const auto& ob_node_ptr : outside_break_nodes) {
                if (this->IsValidConnect(node_ptr, ob_node_ptr, false)) {
                    this->AddEdge(node_ptr, ob_node_ptr);
                } else {
                    this->EraseEdge(node_ptr, ob_node_ptr);
                }
            }
        }
        this->ClearMergedNodesInGraph();
        // Analysisig frontier nodes
        for (const auto& cnode_ptr : near_nav_nodes_) {
            if (this->IsNodeFullyCovered(cnode_ptr)) {
                cnode_ptr->is_frontier = false;
            } else if (!cnode_ptr->is_navpoint) {
                cnode_ptr->is_frontier = true;
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
    bool is_connect = false;
    const float htoler = (node_ptr1->is_odom || node_ptr2->is_odom) ? FARUtil::kTolerZ * 2.0f : FARUtil::kTolerZ;
    if (!is_connect && is_check_contour) {
        if (this->IsBoundaryConnect(node_ptr1, node_ptr2) ||
            (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && this->IsInContourDirConstraint(node_ptr1, node_ptr2))) 
        {
            this->RecordContourEdge(node_ptr1, node_ptr2);
        } else if ((node_ptr1->is_contour_match && node_ptr2->is_contour_match) || !this->IsSameLevelConnct(node_ptr1, node_ptr2, htoler)){
            this->DeleteContourEdge(node_ptr1, node_ptr2);
        }
    }
    this->TopTwoContourConnector(node_ptr1, node_ptr2);
    if (FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects)) {
        if (this->IsConvexConnect(node_ptr1, node_ptr2)) {
            this->RecordPolygonEdge(node_ptr1, node_ptr2, dg_params_.votes_size);
        }
        is_connect = true;
    }
    /* check polygon connections */
    if (!is_connect) {
        bool is_valid_edge = false;
        const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size/3.0f) : dg_params_.votes_size;
        if (this->IsConvexConnect(node_ptr1, node_ptr2) && this->IsSameLevelConnct(node_ptr1, node_ptr2, htoler)) {
            if (this->IsInDirectConstraint(node_ptr1, node_ptr2) && ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2)) {
                is_valid_edge = true;
                this->RecordPolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
            }
        }
        if (!is_valid_edge) this->DeletePolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
        if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
            if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) is_connect = true;
        } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
            node_ptr1->edge_votes.erase(node_ptr2->id);
            node_ptr2->edge_votes.erase(node_ptr1->id);
            // clear potential connections
            FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
            FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
        }
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
    return is_connect;
}

bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    if (FARUtil::IsFreeNavNode(node_ptr) || !node_ptr->is_frontier) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    for (const auto& near_optr : check_odom_list) {
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < FARUtil::kNearDist) return true;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            // TODO: concave nodes will not be marked as covered based on current implementation
            if (FARUtil::IsTypeInStack(node_ptr, near_optr->connect_nodes)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (diff_p.norm() < FARUtil::kNearDist || FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
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
    if (!node_ptr_from->is_frontier && !node_ptr_to->is_frontier) is_cover_connect = true;
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && cnode->is_frontier) continue;
        if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || cnode->is_merged || ref_diff.norm() < FARUtil::kEpsilon) continue;
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
    const Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
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
    const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
    if (topo_dir.norm() > FARUtil::kEpsilon) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
        if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr->vertices, ev_p, FARUtil::free_odom_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}

void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return;
    const int it1c = std::accumulate(it1->second.begin(), it1->second.end(), 0);
    const int it2c = std::accumulate(it2->second.begin(), it2->second.end(), 0);

    const bool is_valid_2to1 = FARUtil::VoteRankInVotes(it1c, node_ptr1->contour_votes) > 1 ? false : true;
    const bool is_valid_1to2 = FARUtil::VoteRankInVotes(it2c, node_ptr2->contour_votes) > 1 ? false : true;

    if (FARUtil::IsVoteTrue(it1->second) && is_valid_2to1 && is_valid_1to2) {
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects))
        {
            // connect contours
            node_ptr1->contour_connects.push_back(node_ptr2);
            node_ptr2->contour_connects.push_back(node_ptr1);
        }
    } else {
        // delete contour connection
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->contour_connects);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->contour_connects);
    }
}

void DynamicGraph::RecordContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
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

void DynamicGraph::RecordPolygonEdge(const NavNodePtr& node_ptr1, 
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
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->connect_nodes)) 
        {
            node_ptr1->connect_nodes.push_back(node_ptr2);
            node_ptr2->connect_nodes.push_back(node_ptr1);
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
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects))
        {
            node_ptr1->contour_connects.push_back(node_ptr2);
            node_ptr2->contour_connects.push_back(node_ptr1);
        }   
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

void DynamicGraph::DeletePolygonEdge(const NavNodePtr& node_ptr1, 
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
void DynamicGraph::DeleteContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
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
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    for (const auto& node_ptr : globalGraphNodes_) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near = false;
        if (FARUtil::IsNodeInLocalRange(node_ptr)) {
            if (FARUtil::IsOutsideGoal(node_ptr)) continue;
            wide_near_nodes_.push_back(node_ptr);
            node_ptr->is_wide_near = true;
            if (this->IsActivateNavNode(node_ptr)) {
                near_nav_nodes_.push_back(node_ptr);
                node_ptr->is_near_nodes = true;
                if (node_ptr->is_navpoint) {
                    node_ptr->position.intensity = node_ptr->fgscore;
                    internav_near_nodes_.push_back(node_ptr);
                    if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist * 2.5) {
                        surround_internav_nodes_.push_back(node_ptr);
                    }
                }
            }
        }
    }
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { // add additional odom connections to wide near stack
        if (!cnode_ptr->is_wide_near && !FARUtil::IsOutsideGoal(cnode_ptr)) {
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
        if (FARUtil::IsTypeInStack(node_ptr, internav_near_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
            return false;
        }
        return true;
    }
    if (FARUtil::IsPointNearNewPoints(node_ptr->position, false)) { // if nearby env changes;
        this->ResetNodeFilters(node_ptr);
        this->ResetNodeConnectVotes(node_ptr);
    }

    if (!node_ptr->is_contour_match) {
        const float ndist = (node_ptr->position - odom_node_ptr_->position).norm();
        if (ndist < MARGIN_DIST) return false;
        else return true;
    }
    else if (node_ptr->is_finalized) return true;

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
