#ifndef CONTOUR_GRAPH_H
#define CONTOUR_GRAPH_H

#include "utility.h"

struct ConnectPair
{
    cv::Point2f start_p;
    cv::Point2f end_p;

    ConnectPair() = default;
    ConnectPair(const cv::Point2f& p1, const cv::Point2f& p2):start_p(p1), end_p(p2) {}
    ConnectPair(const Point3D& p1, const Point3D& p2) {
        this->start_p.x = p1.x;
        this->start_p.y = p1.y;
        this->end_p.x = p2.x;
        this->end_p.y = p2.y;
    }
    
    bool operator ==(const ConnectPair& pt) const 
    {
        return (this->start_p == pt.start_p && this->end_p == pt.end_p) || (this->start_p == pt.end_p && this->end_p == pt.start_p);
    }
};

struct HeightPair
{
    float minH;
    float maxH;
    HeightPair() = default;
    HeightPair(const float& minV, const float& maxV):minH(minV), maxH(maxV) {}
    HeightPair(const Point3D& p1, const Point3D p2) {
        this->minH = std::min(p1.z, p2.z);
        this->maxH = std::max(p1.z, p2.z);
    }
};

struct ContourGraphParams {
    ContourGraphParams() = default;
    float kPillarPerimeter;
};

class ContourGraph {
public:
    ContourGraph() = default;
    ~ContourGraph() = default;

    static CTNodeStack  contour_graph_;
    static std::vector<PointPair> global_contour_;
    static std::vector<PointPair> inactive_contour_;
    static std::vector<PointPair> unmatched_contour_;
    static std::vector<PointPair> boundary_contour_;
    static std::vector<PointPair> local_boundary_;

    void Init(const ContourGraphParams& params);
    
    // static functions
    void UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                            const std::vector<std::vector<Point3D>>& filtered_contours);

    /* Match current contour with global navigation nodes */
    void MatchContourWithNavGraph(const NodePtrStack& global_nodes,
                                  const NodePtrStack& near_nodes,
                                  CTNodeStack& new_convex_vertices);

    void ExtractGlobalContours();

    static NavNodePtr MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes);

    static bool IsContourLineMatch(const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode);
    
    static bool IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, 
                                             const NavNodePtr& node_ptr2);

    static bool IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, 
                                            const CTNodePtr& ctnode2);

    static bool IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1,
                                             const NavNodePtr& node_ptr2);

    static bool IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr,
                                              const NavNodePtr& goal_ptr);

    static bool IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2);

    static bool IsEdgeCollideBoundary(const Point3D& p1, const Point3D& p2);

    static bool IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                           const ConnectPair& bd_cedge,
                                           const HeightPair h_pair,
                                           const bool& is_global_check);
    
    static inline void MatchCTNodeWithNavNode(const CTNodePtr& ctnode_ptr, const NavNodePtr& node_ptr) {
        if (ctnode_ptr == NULL || node_ptr == NULL) return;
        ctnode_ptr->is_global_match = true;
        ctnode_ptr->nav_node_id = node_ptr->id;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->is_contour_match = true;
    }

    static bool ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius);

    static void AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    static void DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsPointInVetexAngleRestriction(const CTNodePtr& ctnode, const Point3D end_p);

    void ResetCurrentContour();

private:

    static CTNodeStack polys_ctnodes_;
    static PolygonStack contour_polygons_;
    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    NavNodePtr odom_node_ptr_ = NULL;
    bool is_robot_inside_poly_ = false;

    // global contour set
    static std::unordered_set<NavEdge, navedge_hash> global_contour_set_;
    static std::unordered_set<NavEdge, navedge_hash> boundary_contour_set_;

    
    /* static private functions */
    inline void AddCTNodeToGraph(const CTNodePtr& ctnode_ptr) {
        if (ctnode_ptr == NULL && ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add ctnode to contour graph fails, ctnode is invaild.");
            return;
        }
        ContourGraph::contour_graph_.push_back(ctnode_ptr);
    }

    inline void AddPolyToContourPolygon(const PolygonPtr& poly_ptr) {
        if (poly_ptr == NULL || poly_ptr->vertices.empty()) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add polygon fails, polygon is invaild.");
            return;
        }
        ContourGraph::contour_polygons_.push_back(poly_ptr);
    } 

    inline bool IsActiveEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_active && node_ptr2->is_active) {
            return true;
        }
        return false;
    }
    
    inline void AddConnect(const CTNodePtr& ctnode_ptr1, const CTNodePtr& ctnode_ptr2) {
        if (ctnode_ptr1 != ctnode_ptr2 &&
            !FARUtil::IsTypeInStack(ctnode_ptr2, ctnode_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(ctnode_ptr1, ctnode_ptr2->connect_nodes))
        {
            ctnode_ptr1->connect_nodes.push_back(ctnode_ptr2);
            ctnode_ptr2->connect_nodes.push_back(ctnode_ptr1);
        }
    }

    template <typename NodeType1, typename NodeType2>
    static inline bool IsInMatchHeight(const NodeType1& node_ptr1, const NodeType2& node_ptr2) {
        if (abs(node_ptr1->position.z - node_ptr2->position.z) < FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }

    static inline bool IsEdgeOverlapInHeight(const HeightPair& cur_hpair, HeightPair ref_hpair, const bool is_extend=true) {
        if (is_extend) {
            ref_hpair.minH -= FARUtil::kTolerZ, ref_hpair.maxH += FARUtil::kTolerZ;
        }
        if (cur_hpair.maxH < ref_hpair.minH || cur_hpair.minH > ref_hpair.maxH) {
            return false;
        }
        return true;
    }

    static inline bool IsEdgeInLocalRange(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_near_nodes || node_ptr2->is_near_nodes || FARUtil::IsNodeInLocalRange(node_ptr1) || FARUtil::IsNodeInLocalRange(node_ptr2)) {
            return true;
        }
        return false;
    }

    template <typename NodeType>
    static inline cv::Point2f NodeProjectDir(const NodeType& node) {
        cv::Point2f project_dir(0,0);
        if (node->free_direct != NodeFreeDirect::PILLAR && node->free_direct != NodeFreeDirect::UNKNOW) {
            const Point3D topo_dir = FARUtil::SurfTopoDirect(node->surf_dirs);
            if (node->free_direct == NodeFreeDirect::CONCAVE) {
                project_dir = cv::Point2f(topo_dir.x, topo_dir.y);
            } else {
                project_dir = cv::Point2f(-topo_dir.x, -topo_dir.y);
            }
        }
        return project_dir;
    }

    template <typename NodeType>
    static inline cv::Point2f ProjectNode(const NodeType& node, const float& dist) {
        const cv::Point2f node_cv = cv::Point2f(node->position.x, node->position.y);
        const cv::Point2f dir = NodeProjectDir(node);
        return node_cv + dist * dir;
    }

    static inline void RemoveMatchWithNavNode(const NavNodePtr& node_ptr) {
        if (!node_ptr->is_contour_match) return;
        node_ptr->ctnode->is_global_match = false;
        node_ptr->ctnode->nav_node_id = 0;
        node_ptr->ctnode = NULL;
        node_ptr->is_contour_match = false;
    }

    /**
     * @brief extract necessary ctnodes that are essential for contour construction
     */
    void EnclosePolygonsCheck();

    CTNodePtr FirstMatchedCTNode(const CTNodePtr& ctnode_ptr);

    void UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p);

    static bool IsCTNodesConnectWithinOrder(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2,
                                            CTNodePtr& block_vertex);

    static ConnectPair ReprojectEdge(const NavNodePtr& node1, const NavNodePtr& node2, const float& dist, const bool& is_global_check);

    static ConnectPair ReprojectEdge(const CTNodePtr& node1, const NavNodePtr& node2, const float& dist);

    static bool IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge);

    static bool IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge);

    static bool IsCTMatchLineFreePolygon(const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check);

    static bool IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new);

    inline static bool IsNeedGlobalCheck(const Point3D& p1, const Point3D& p2) {
        if (!FARUtil::IsPointInLocalRange(p1) || !FARUtil::IsPointInLocalRange(p2)) {
            return true;
        }
        return false;
    }

    static inline bool IsOverlapRange(const HeightPair& hpair, const HeightPair& hpairRef) {
        if (hpair.maxH > hpairRef.minH - FARUtil::kTolerZ || hpair.minH < hpairRef.maxH + FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }

    inline void ClearContourGraph() {
        ContourGraph::polys_ctnodes_.clear();
        ContourGraph::contour_graph_.clear();
        ContourGraph::contour_polygons_.clear(); 
    }

    bool IsAPillarPolygon(const PointStack& vertex_points, float& perimeter);

    void CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar);

    void CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr);

    NavNodePtr NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes);

    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    
    /* Analysis CTNode surface angle */
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);


};



#endif