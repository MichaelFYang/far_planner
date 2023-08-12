#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include "utility.h"
#include "contour_graph.h"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;

// create hash for Marker
struct MarkerHash {
    std::size_t operator()(const Marker& marker) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, marker.type);
        boost::hash_combine(seed, marker.ns);
        boost::hash_combine(seed, marker.id);
        return seed;
    }
};

enum VizColor {
    RED     = 0,
    ORANGE  = 1,
    BLACK   = 2,
    YELLOW  = 3,
    BLUE    = 4,
    GREEN   = 5,
    EMERALD = 6,
    WHITE   = 7,
    MAGNA   = 8,
    PURPLE  = 9
};

class DPVisualizer {
private:
    rclcpp::Node::SharedPtr nh_;
    // Utility Cloud 
    PointCloudPtr point_cloud_ptr_;
    // rviz publisher 
    rclcpp::Publisher<Marker>::SharedPtr viz_path_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr viz_node_pub_, viz_graph_pub_, viz_poly_pub_, viz_contour_pub_, viz_map_pub_, viz_view_extend;

    // marker set for nodes visualization based on MarkerHash
    std::unordered_set<Marker, MarkerHash> marker_set_;
public:
    DPVisualizer() = default;
    ~DPVisualizer() = default;

    void Init(const rclcpp::Node::SharedPtr nh);

    void VizNodes(const NodePtrStack& node_stack, 
                  const std::string& ns,
                  const VizColor& color,
                  const float scale=0.75f,
                  const float alpha=0.75f);

    void VizGlobalPolygons(const std::vector<PointPair>& contour_pairs, 
                           const std::vector<PointPair>& unmatched_pairs);

    void VizViewpointExtend(const NavNodePtr& ori_nav_ptr, const Point3D& extend_point);

    // True for non-attempts path
    void VizPath(const NodePtrStack& global_path, const bool& is_free_nav=false);

    void VizMapGrids(const PointStack& neighbor_centers, 
                     const PointStack& occupancy_centers,
                     const float& ceil_length,
                     const float& ceil_height);

    void VizContourGraph(const CTNodeStack& contour_graph);

    void VizPoint3D(const Point3D& point, 
                    const std::string& ns,
                    const VizColor& color,
                    const float scale=1.0f,
                    const float alpha=0.9f);

    void VizGraph(const NodePtrStack& graph);

    void VizPointCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viz_pub, 
                       const PointCloudPtr& pc);

    inline void PubNodesVisualization() {
        MarkerArray marker_array;
        for (auto& marker : marker_set_) {
            marker_array.markers.push_back(marker);
        }
        viz_node_pub_->publish(marker_array);
        marker_set_.clear();
    }

    static void SetMarker(const rclcpp::Node::SharedPtr nh,
                          const VizColor& color, 
                          const std::string& ns,
                          const float& scale, 
                          const float& alpha, 
                          Marker& scan_marker,
                          const float& scale_ratio=FARUtil::kVizRatio);

    static void SetColor(const VizColor& color, const float& alpha, Marker& scan_marker);

};

#endif