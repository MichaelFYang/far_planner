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
    float kAroundDist;
    float kPillarPerimeter;
    float kMatchDist;
};

class ContourGraph {
public:
    ContourGraph() = default;
    ~ContourGraph() = default;

    static CTNodeStack  contour_graph_;
    static std::vector<PointPair> global_contour_;
    static std::vector<PointPair> inactive_contour_;

    void Init(const ContourGraphParams& params);
    
    // static functions
    void UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                            const std::vector<std::vector<Point3D>>& filtered_contours);

    /* Match current contour with global navigation nodes */
    void MatchContourWithNavGraph(const NodePtrStack& nav_graph,
                                  CTNodeStack& new_convex_vertices);

    void ExtractGlobalContours(const NodePtrStack& nav_graph);

    static bool IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, 
                                          const NavNodePtr& node_ptr2);

    static bool IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, 
                                            const CTNodePtr& ctnode2);

    static bool IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1,
                                             const NavNodePtr& node_ptr2,
                                             const bool& is_local_only=false);

    static bool IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr,
                                              const NavNodePtr& goal_ptr);

    static bool IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2);

    static bool IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                           const bool& is_global_check);

    static bool ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius);

    bool IsPointInVetexAngleRestriction(const CTNodePtr& ctnode, const Point3D end_p);

private:

    static PolygonStack contour_polygons_;
    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    NavNodePtr odom_node_ptr_ = NULL;
    bool is_robot_inside_poly_ = false;
    
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

    inline void AddConnect(const CTNodePtr& ctnode_ptr1, const CTNodePtr& ctnode_ptr2) {
        if (ctnode_ptr1 != ctnode_ptr2 &&
            !FARUtil::IsTypeInStack(ctnode_ptr2, ctnode_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(ctnode_ptr1, ctnode_ptr2->connect_nodes))
        {
            ctnode_ptr1->connect_nodes.push_back(ctnode_ptr2);
            ctnode_ptr2->connect_nodes.push_back(ctnode_ptr1);
        }
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

    void UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p);

    static inline ConnectPair ReprojectEdge(const NavNodePtr& node1, const NavNodePtr& node2, const float& dist);

    static bool IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge);

    static bool IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge);

    inline static bool IsNeedGlobalCheck(const Point3D& p1, const Point3D& p2, const Point3D& robot_p) {
        if ((p1 - robot_p).norm_flat() > FARUtil::kSensorRange || (p2 - robot_p).norm_flat() > FARUtil::kSensorRange) {
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
        ContourGraph::contour_graph_.clear();
        ContourGraph::contour_polygons_.clear(); 
    }

    bool IsAPillarPolygon(const PointStack& vertex_points);

    void CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar);

    void CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr);

    NavNodePtr NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& nav_graph);

    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    
    /* Analysis CTNode surface angle */
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);


};



#endif