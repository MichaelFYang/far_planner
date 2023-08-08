#ifndef FAR_UTILITY_H
#define FAR_UTILITY_H

/* C++ Library */
#include <memory>
#include <string>
#include <time.h>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <boost/functional/hash.hpp>

/*Internal Library*/
#include "point_struct.h"
#include "node_struct.h"
#include "grid.h"

/*ROS2 Library*/
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

// Need to convert visibility_graph_msg
#include <visibility_graph_msg/msg/graph.hpp>
#include <visibility_graph_msg/msg/node.hpp>

/*OpenCV Library*/
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/* PCL Library*/
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

/* PCL + ROS2 Library*/
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

/* Utility Header Timer */
#include "time_measure.h"

typedef pcl::KdTreeFLANN<PCLPoint>::Ptr PointKdTreePtr;
typedef std::vector<Point3D> PointStack;
typedef std::vector<cv::Point2f> CVPointStack;
typedef std::vector<NavNodePtr> NodePtrStack;

typedef std::vector<std::size_t> IdxStack;
typedef std::unordered_set<std::size_t> IdxSet;
typedef std::unordered_map<std::size_t, std::size_t> IdxMap;

enum AngleNoiseDirect {
    NARROW = 1,
    WIDE =  -1,
    NO_NOISE = 0
};

class FARUtil {
public:
    FARUtil() = default;
    ~FARUtil() = default;
    /* Static Constant Values*/ 
    static const float  kEpsilon;
    static const float  kINF;
    static bool  IsStaticEnv;
    static bool  IsDebug;
    static bool  IsMultiLayer;
    static Point3D robot_pos;
    static Point3D odom_pos;
    static Point3D map_origin;
    static Point3D free_odom_p; 
    static float robot_dim;
    static float kAngleNoise;
    static float kCellLength;
    static float kCellHeight;
    static float vehicle_height;
    static float kLeafSize;
    static float kHeightVoxel;
    static float kNavClearDist;
    static float kNearDist;
    static float kMatchDist;
    static float kProjectDist;
    static float kNewPIThred;
    static float kSensorRange;
    static float kMarginDist;
    static float kLocalPlanRange;
    static float kMarginHeight;
    static float kTerrainRange;
    static float kFreeZ;
    static float kObsDecayTime;
    static float kNewDecayTime;
    static int   kDyObsThred;
    static int   KNewPointC;
    static int   kObsInflate;
    static float kTolerZ;
    static float kAcceptAlign;
    static float kVizRatio;
    static double systemStartTime;
    static TimeMeasure Timer;
    static std::string worldFrameId;
    // PCL Clouds
    static PointCloudPtr surround_obs_cloud_;   // surround obstacle cloud
    static PointCloudPtr surround_free_cloud_;  // surround free space cloud
    static PointCloudPtr stack_new_cloud_;      // new obstacle points cloud      
    static PointCloudPtr stack_dyobs_cloud_;
    static PointCloudPtr cur_new_cloud_;
    static PointCloudPtr cur_dyobs_cloud_;
    static PointCloudPtr cur_scan_cloud_;
    static PointCloudPtr local_terrain_obs_;
    static PointCloudPtr local_terrain_free_;
    // kdTree cloud
    static PointKdTreePtr kdtree_new_cloud_;
    static PointKdTreePtr kdtree_filter_cloud_;

    /*
    Input: New sensor input cloud and Cached surround cloud
    Output: Update new_cloud_
    */
    static void ExtractNewObsPointCloud(const PointCloudPtr& cloudIn,
                                        const PointCloudPtr& cloudRefer,
                                        const PointCloudPtr& cloudNew);

    static void ResetCloudIntensity(const PointCloudPtr& cloudIn, const bool isHigh);

    static void ResetCloudIntensityWithTime(const PointCloudPtr& cloudInOut, const rclcpp::Node::SharedPtr nh);
    
    static void CropPCLCloud(const PointCloudPtr& cloudIn,
                             const PointCloudPtr& cloudCropped,
                             const Point3D& centriod,
                             const float& range);
    
    static void CropPCLCloud(const PointCloudPtr& cloudInOut,
                             const Point3D& centriod,
                             const float& range);

    static void CropBoxCloud(const PointCloudPtr& cloudInOut, const Point3D& center_p, const Point3D& crop_size);

    static void ClusterFilterCloud(const PointCloudPtr& cloudInOut, 
                                   const float& radius,
                                   const std::size_t c_thred);

    static void CropCloudWithinHeight(const PointCloudPtr& cloudInOut, const float& height);

    static void StackCloudByTime(const PointCloudPtr& curInCloud,
                                 const PointCloudPtr& StackCloud,
                                 const float& duration,
                                 const rclcpp::Node::SharedPtr nh);

    static void TransferCloud(const Point3D& transPoint,
                              const PointCloudPtr& cloudInOut);
         
    static void ConvertCloudToPCL(const PointStack& point_stack, 
                                  const PointCloudPtr& point_cloud_ptr); 

    static void ConvertCTNodeStackToPCL(const CTNodeStack& ctnode_stack, 
                                        const PointCloudPtr& point_cloud_ptr);

    static void FilterCloud(const PointCloudPtr& point_cloud, const float& leaf_size);

    static void FilterCloud(const PointCloudPtr& point_cloud, const Eigen::Vector3d& leaf_size);

    static void TransformPCLFrame(const std::string& from_frame_id,
                                  const std::string& to_frame_id,
                                  const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                  const PointCloudPtr& cloudInOut);

    static void TransformPoint3DFrame(const std::string& from_frame_id,
                                      const std::string& to_frame_id,
                                      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                      Point3D& point);

    static bool IsSameFrameID(const std::string& cur_frame, const std::string& ref_frame);
    
    static void RemoveNanInfPoints(const PointCloudPtr& cloudInOut);

    static bool IsPointNearNewPoints(const Point3D& p, const bool& is_creation=false);

    static bool IsPointVisiableFromNode(const Point3D& p, const NavNodePtr& node_ptr);

    static std::size_t PointInXCounter(const Point3D& p,
                                       const float& radius,
                                       const PointKdTreePtr& KdTree);

    static std::size_t PointInNewCounter(const Point3D& p, const float& radius);

    template <typename T>
    static bool IsTypeInStack(const T& elem, const std::vector<T>& T_stack) {
        if (std::find(T_stack.begin(), T_stack.end(), elem) != T_stack.end()) {
            return true;
        }
        return false;
    }

    static void Flat3DPointCloud(const PointCloudPtr& cloudIn, 
                                 const PointCloudPtr& cloudFlat,
                                 const bool& is_downsample=true);

    static void ExtractFreeAndObsCloud(const PointCloudPtr& newCloudIn,
                                       const PointCloudPtr& freeCloudOut,
                                       const PointCloudPtr& obsCloudOut);

    static void UpdateKdTrees(const PointCloudPtr& newObsCloud);

    static void ClearKdTree(const PointCloudPtr& cloud_ptr,
                            const PointKdTreePtr& kdTree_ptr);

    static void SetDifference(std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& diff);

    template <typename Point>
    static bool IsPointInToleratedHeight(const Point& p, const float& height=FARUtil::kTolerZ) {
        if (abs(p.z - FARUtil::robot_pos.z) < height) return true;
        return false;
    }

    template <typename _T>
    static int Signum(const _T x) {
        return x == 0 ? 0 : x < 0 ? -1 : 1;
    }

    static PCLPoint Point3DToPCLPoint(const Point3D& point);

    static geometry_msgs::msg::Point Point3DToGeoMsgPoint(const Point3D& point);

    static int Mod(const int& a, const int& b);

    static bool IsSamePoint3D(const Point3D& p1, const Point3D& p2);

    static void EraseNodeFromStack(const NavNodePtr& node_ptr,
                                   NodePtrStack& node_stack);

    static float NoiseCosValue(const float& dot_value, const bool& is_large, const float& noise = FARUtil::kAngleNoise);

    static void CopyGroundPCL(const PointCloudPtr& freeCloudIn,
                              const PointCloudPtr& freeCloudOut);

    static bool IsOutReducedDirs(const Point3D& diff_p,
                                 const PointPair& surf_dirs);

    static bool IsOutReducedDirs(const Point3D& diff_p,
                                 const NavNodePtr& node_ptr);

    static bool IsInCoverageDirPairs(const Point3D& diff_p,
                                     const NavNodePtr& node_ptr);

    static bool IsInContourDirPairs(const Point3D& diff_p,
                                    const PointPair& surf_dirs);

    static float MarginAngleNoise(const float& dist, 
                                  const float& max_dist,
                                  const float& angle_noise);

    static bool IsVoteTrue(const std::deque<int>& votes, const bool& is_balance=true);

    static int VoteRankInVotes(const int& c, const std::vector<int>& ordered_votes);

    static bool IsDirsConverage(const PointPair& cur_directs,
                                const PointPair& update_directs);

    static void InflateCloud(const PointCloudPtr& obsCloudInOut,
                             const float& resol,
                             const int& inflate_size,
                             const bool& deep_z_inflate);

    static void SortEdgesClockWise(const Point3D& center, std::vector<PointPair>& edges);

    static bool ClockwiseLess(const Point3D& p1, const Point3D& p2);

    static void ClockwiseTwoPoints(const Point3D& center, PointPair& edge);

    static void ExtractOverlapCloud(const PointCloudPtr& cloudIn,
                                    const PointCloudPtr& cloudRef,
                                    const PointCloudPtr& cloudOverlapOut,
                                    const float& margin_ratio=1.0);

    static void RemoveOverlapCloud(const PointCloudPtr& cloudInOut,
                                   const PointCloudPtr& cloudRef,
                                   const bool& is_copy_cloud=false);

    static void RemoveIndicesFromCloud(const pcl::PointIndices::Ptr& outliers,
                                       const PointCloudPtr& cloudInOut);

    static float ApproxAtan2(const float& y, const float x);

    static float CosinTheta(const Point3D& vertex, const Point3D& p1, const Point3D& p2);

    static float PixelDistance(const cv::Point2f& pre_p, const cv::Point2f& cur_p);

    static float DirsDistance(const PointPair& ref_dirs, const PointPair& compare_dirs);

    static Point3D SurfTopoDirect(const PointPair& dirs, bool& _is_wall);

    static Point3D SurfTopoDirect(const PointPair& dirs);

    template <typename NodeType1, typename NodeType2>
    static inline bool IsAtSameLayer(const NodeType1& node_ptr1, const NodeType2& node_ptr2) {
        if (FARUtil::IsMultiLayer && abs(node_ptr1->position.z - node_ptr2->position.z) > FARUtil::kTolerZ) {
            return false;
        }
        return true;
    }

    template <typename T_vec>
    static Point3D AveragePoints(const T_vec& point_stack) {
        Point3D mean_p(0,0,0);
        if (point_stack.empty()) {
            std::cout << "FARUtil: Averaging points fails, stack is empty" << std::endl;
            return mean_p;
        }
        for (const auto& pos : point_stack) {
            mean_p = mean_p + pos;
        }
        return mean_p / (float)point_stack.size();
    }

    static PointPair AverageDirs(const std::vector<PointPair>& dirs_stack);

    static std::size_t CounterOfPillar(const std::deque<PointPair>& dirs_stack);

    static float DistanceToLineSeg2D(const Point3D& p, const PointPair& line);

    static void CorrectDirectOrder(const PointPair& ref_dir, PointPair& dirInOUt);

    static void CreatePointsAroundCenter(const Point3D& center_p, 
                                         const float& radius,
                                         const float& resol,
                                         PointStack& points_stack,
                                         const bool& is_sort=true);

    static Point3D RANSACPoisiton(const std::deque<Point3D>& pos_filter_stack, const float& margin, std::size_t& inlier_size);

    static PointPair RANSACSurfDirs(const std::deque<PointPair>& surf_dirs_stack, const float& margin, std::size_t& inlier_size);
    
    static float VerticalDistToLine2D(const Point3D& start_p, 
                                      const Point3D& end_p, 
                                      const Point3D& cur_p);

    static bool IsInCylinder(const Point3D& start_p, 
                             const Point3D& end_p, 
                             const Point3D& cur_p,
                             const float& radius,
                             const bool& is_2D=false);

    static float LineMatchPercentage(const PointPair& line1, const PointPair& line2);

    static Point3D ContourSurfDirs(const Point3D& end_p, 
                                   const Point3D& start_p, 
                                   const Point3D& center_p,
                                   const float& radius);

    static bool IsOutReachNode(const NavNodePtr& node_ptr);

    static bool IsPointInLocalRange(const Point3D& p, const bool& is_large_h=false);

    static bool IsPointInMarginRange(const Point3D& p);

    static bool IsNodeInLocalRange(const NavNodePtr& node_ptr, const bool& is_large_h=false) {
        return IsPointInLocalRange(node_ptr->position, is_large_h);
    }

    static bool IsNodeInExtendMatchRange(const NavNodePtr& node_ptr) {
        if (FARUtil::IsPointInToleratedHeight(node_ptr->position, FARUtil::kTolerZ * 1.5f) && (node_ptr->position - FARUtil::odom_pos).norm() < FARUtil::kSensorRange) {
            return true;
        }
        return false;
    }

    static bool IsFreeNavNode(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || node_ptr->is_navpoint) return true;
        return false;
    }

    static bool IsStaticNode(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || node_ptr->is_goal) return true;
        return false;
    }

    static bool IsOutsideGoal(const NavNodePtr& node_ptr) {
        if (node_ptr->is_goal && !node_ptr->is_navpoint) return true;
        return false;
    }

    static float ClampAbsRange(const float& value, float range) {
        range = abs(range);
        return std::min(std::max(-range, value), range);
    }

    template <typename Point>
    static Point NormalizePXY(const Point& p) {
        Point return_p = p;
        const float norm = std::hypotf(return_p.x, return_p.y);
        if (norm > FARUtil::kEpsilon) {
            return_p.x /= norm, return_p.y /= norm;
        } else {
            if (FARUtil::IsDebug) std::cout << "FARUtil: Point XY normalization fails, vector norm is too small." << std::endl;
        }
        return return_p;
    }
    
    template <typename Point>
    static bool PointInsideAPoly(const std::vector<Point>& poly, const Point& p) {
        // By Randolph Franklin, https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
        int i, j, c = 0;
        int npol = poly.size();
        if (npol < 3) {
            if (FARUtil::IsDebug) std::cout << "FARUtil: The vertices number of a polygon is less than 3." <<std::endl;
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
    static bool IsConvexPoint(const PolygonPtr& poly_ptr, const Point& ev_p) {
        if (FARUtil::PointInsideAPoly(poly_ptr->vertices, ev_p) != poly_ptr->is_robot_inside) {
            return true;
        } 
        return false;
    }
    

};



#endif