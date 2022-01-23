#ifndef GRAPH_EXTRACTOR_H
#define GRAPH_EXTRACTOR_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <std_msgs/String.h>
#include <visibility_graph_msg/Graph.h>
#include <visibility_graph_msg/Node.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "boundary_handler/point_struct.h"

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

typedef std::pair<Point3D, Point3D> PointPair;
typedef std::vector<Point3D> PointStack;

enum VizColor {
    RED = 0,
    ORANGE = 1,
    BLACK = 2,
    YELLOW = 3,
    BLUE = 4,
    GREEN = 5,
    EMERALD = 6,
    WHITE = 7,
    MAGNA = 8,
    PURPLE = 9
};

enum NodeFreeDirect {
  UNKNOW  =  0,
  CONVEX  =  1,
  CONCAVE =  2,
  PILLAR  =  3
};

struct Polygon
{
  Polygon() = default;
  std::size_t N;
  std::vector<Point3D> vertices;
  bool is_robot_inside;
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct NavNode {
    NavNode() = default;
    std::size_t id;
    Point3D position;
    NodeFreeDirect free_direct;
    PointPair surf_dirs;
    PolygonPtr poly_ptr;

    bool is_covered;
    bool is_frontier;
    bool is_navpoint;
    bool is_boundary;
    std::vector<std::size_t> connect_idxs;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;

    std::vector<std::size_t> poly_idxs;
    std::vector<std::shared_ptr<NavNode>> poly_connects;

    std::vector<std::size_t> contour_idxs;
    std::vector<std::shared_ptr<NavNode>> contour_connects;

    std::vector<std::size_t> traj_idxs;
    std::vector<std::shared_ptr<NavNode>> traj_connects;
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::vector<NavNodePtr> NodePtrStack;

struct GraphExtractorParams {
    GraphExtractorParams() = default;
    std::string frame_id;
    std::string vgraph_path;
    std::string bd_file_path;
    std::string traj_file_path;
    float viz_scale_ratio;
    float height_tolz;
};

class GraphExtractor {
public:
    GraphExtractor()  = default;
    ~GraphExtractor() = default;

    void Init();
    void Run();
    void Loop();

private:
    ros::NodeHandle nh;
    ros::Publisher  graph_viz_pub_, viz_node_pub_;

    std::size_t id_ = 0;

    GraphExtractorParams ge_params_;
    PolygonStack extracted_polys_;
    NodePtrStack extracted_graph_;
    MarkerArray graph_marker_array_;
    std::size_t robot_id_;

    Point3D free_p_;
    PointStack traj_;

    PointCloudPtr boundary_ptr_;

    void LoadParmas();

    void ConstructVGraph(const PolygonStack& polysIn, NodePtrStack& graphOut);

    void SetMarker(const VizColor& color, 
                   const std::string& ns,
                   const float scale,
                   const float alpha, 
                   Marker& scan_marker);
    
    void SetColor(const VizColor& color, const float alpha, Marker& scan_marker);

    void ReadBoundaryFile(const PointCloudPtr& bd_cloud, PolygonStack& polysOut);

    void ReadTrajFile(Point3D& free_p);

    void FilterPolyBoundary(const PointCloudPtr& bd_cloud, PolygonStack& polysOut);

    bool IsValidConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void AnalysisConvexity(const NavNodePtr& node_ptr, const PolygonPtr& poly_ptr);

    bool IsOutReducedDirs(const Point3D& diff_p, const PointPair& surf_dirs);

    bool IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsEdgeCollideSegment(const PointPair& line, const PointPair& edge);

    PointPair ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist);

    /**
     * @brief Save the current vgraph to a vgh file
     * 
     * @param graphIn current stored v-graph
     */
    void SaveVGraph(const NodePtrStack& graphIn);

    void VisualizeGraph(const NodePtrStack& graphIn);

    void VisualizeFreePoint(const Point3D& free_p);

    void CreateNavNode(const Point3D& p, NavNodePtr& node_ptr);

    bool IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);


    /**
     * @brief encode graph to ros graph message
     * 
     * @param graphIn current stored v-graph
     * @param graphOut [out] the encoded v-graph message
     */
    void EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::Graph& graphOut);

    template <typename Point>
    geometry_msgs::Point inline ToGeoMsgP(const Point& p) {
        geometry_msgs::Point geo_p;
        geo_p.x = p.x;
        geo_p.y = p.y; 
        geo_p.z = p.z;
        return geo_p;
    }

    template <typename T>
    inline bool IsTypeInStack(const T& elem, const std::vector<T>& T_stack) {
        if (std::find(T_stack.begin(), T_stack.end(), elem) != T_stack.end()) {
            return true;
        }
        return false;
    }

    inline void ConnectBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        if (this->IsTypeInStack(node_ptr2->id, node_ptr1->contour_idxs)) return;
        node_ptr1->contour_idxs.push_back(node_ptr2->id);
        node_ptr2->contour_idxs.push_back(node_ptr1->id);
        node_ptr1->contour_connects.push_back(node_ptr2);
        node_ptr2->contour_connects.push_back(node_ptr1);
    }

    inline void ConnectVEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        // poly connects
        if (!this->IsTypeInStack(node_ptr2->id, node_ptr1->poly_idxs)) {
            node_ptr1->poly_idxs.push_back(node_ptr2->id);
            node_ptr2->poly_idxs.push_back(node_ptr1->id);
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
        // graph connects
        if (!this->IsTypeInStack(node_ptr2->id, node_ptr1->connect_idxs)) {
            node_ptr1->connect_idxs.push_back(node_ptr2->id);
            node_ptr2->connect_idxs.push_back(node_ptr1->id);
            node_ptr1->connect_nodes.push_back(node_ptr2);
            node_ptr2->connect_nodes.push_back(node_ptr1);
        }
    }

    inline void ResetGraph(NodePtrStack& graphOut) {
        graphOut.clear();
    }

    inline int Mod(const int& a, const int& b) {
        return (b + (a % b)) % b;
    }

    inline void CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr, const Point3D& free_p) {
        poly_ptr = std::make_shared<Polygon>();
        poly_ptr->N = poly_points.size();
        poly_ptr->vertices = poly_points;
        poly_ptr->is_robot_inside = this->PointInsideAPoly(poly_points, free_p);
    }

    inline void ConvertGraphToMsg(const NodePtrStack& graph, visibility_graph_msg::Graph& graph_msg) {
        graph_msg.header.frame_id = ge_params_.frame_id;
        graph_msg.header.stamp = ros::Time::now();
        graph_msg.robot_id = robot_id_;
        EncodeGraph(graph, graph_msg);
    }

    inline bool AddNodePtrToGraph(const NavNodePtr& node_ptr, NodePtrStack& graphOut) {
        if (node_ptr != NULL) {
            graphOut.push_back(node_ptr);
            return true;
        }
        return false;
    }

    inline bool IsConvexConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->free_direct != NodeFreeDirect::CONCAVE && 
            node_ptr2->free_direct != NodeFreeDirect::CONCAVE)
        {
            return true;
        }
        return false;
    }

    inline Point3D SurfTopoDirect(const PointPair& dirs, bool& _is_wall) {
        const Point3D topo_dir = dirs.first + dirs.second;
        _is_wall = false;
        if (topo_dir.norm_flat() > 1e-7) {
            return topo_dir.normalize_flat();
        } else {
            _is_wall = true;
            return Point3D(0,0,0);
        }
    }

    template <typename Point>
    inline bool PointInsideAPoly(const std::vector<Point>& poly, const Point& p) {
        // By Randolph Franklin, https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
        int i, j, c = 0;
        int npol = poly.size();
        if (npol < 3) {
            ROS_WARN("The vertices number of a polygon is less than 3.");
            return false;
        }
        for (i = 0, j = npol-1; i < npol; j = i++) {
            const Point vetex1 = poly[i];
            const Point vetex2 = poly[j];
        if ((((vetex1.y <= p.y) && (p.y < vetex2.y)) ||
             ((vetex2.y <= p.y) && (p.y < vetex1.y))) &&
            (p.x < (vetex2.x - vetex1.x) * (p.y - vetex1.y) / (vetex2.y - vetex1.y) + vetex1.x))
            c = !c;
        }
        return c;
    }

    template <typename Point>
    inline bool IsConvexPoint(const std::vector<Point>& poly, const Point& ev_p, const Point& free_p) {
        const bool in_or_out = this->PointInsideAPoly(poly, free_p);
        if (!this->PointInsideAPoly(poly, ev_p) == in_or_out) {
            return true;
        } 
        return false;
    }
    
};

#endif