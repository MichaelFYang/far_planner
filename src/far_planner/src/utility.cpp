/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/utility.h"

/***************************************************************************************/

void FARUtil::FilterCloud(const PointCloudPtr& point_cloud, const Eigen::Vector3d& leaf_size) {
  // filter point cloud with constant leaf size 0.2m
  pcl::PointCloud<PCLPoint> filter_rs_cloud;
  pcl::VoxelGrid<PCLPoint> vg;
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(leaf_size.x(), leaf_size.y(), leaf_size.z());
  vg.filter(filter_rs_cloud);
  *point_cloud = filter_rs_cloud;
}

void FARUtil::FilterCloud(const PointCloudPtr& point_cloud, const float& leaf_size) {
  // filter point cloud with constant leaf size 0.2m
  const Eigen::Vector3d leaf(leaf_size, leaf_size, leaf_size);
  FilterCloud(point_cloud, leaf);
}

void FARUtil::RemoveNanInfPoints(const PointCloudPtr& cloudInOut) {
  pcl::PointCloud<PCLPoint> temp_cloud;
  temp_cloud.resize(cloudInOut->points.size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      if (FARUtil::IsDebug) ROS_WARN_ONCE("FARUtil: nan or inf point detected.");
      continue;
    }
    temp_cloud.points[idx] = p;
    idx ++;
  }
  temp_cloud.points.resize(idx);
  *cloudInOut = temp_cloud;
}

PCLPoint FARUtil::Point3DToPCLPoint(const Point3D& point) {
  PCLPoint pcl_point;
  pcl_point.x = point.x;
  pcl_point.y = point.y;
  pcl_point.z = point.z;
  pcl_point.intensity = point.intensity;
  return pcl_point;
}

void FARUtil::ExtractNewObsPointCloud(const PointCloudPtr& cloudIn,
                                      const PointCloudPtr& cloudRefer,
                                      const PointCloudPtr& cloudNew)
{
  PointCloudPtr temp_new_cloud(new pcl::PointCloud<PCLPoint>());
  FARUtil::ResetCloudIntensity(cloudIn, false);
  FARUtil::ResetCloudIntensity(cloudRefer, true);
  cloudNew->clear(), temp_new_cloud->clear();
  *temp_new_cloud = *cloudIn + *cloudRefer;
  FARUtil::FilterCloud(temp_new_cloud, FARUtil::kLeafSize * 2.0);
  for (const auto& p : temp_new_cloud->points) {
    if (p.intensity < FARUtil::kNewPIThred) {
      cloudNew->points.push_back(p);
    } 
  }
}

void FARUtil::ResetCloudIntensity(const PointCloudPtr& cloudIn, const bool isHigh) {
  const float kValue = isHigh? 255.0 : 0.0;
  for (std::size_t i=0; i<cloudIn->size(); i++) {
    cloudIn->points[i].intensity = kValue;
  }
}

void FARUtil::ResetCloudIntensityWithTime(const PointCloudPtr& cloudInOut) {
  const float curTime = ros::Time::now().toSec() - FARUtil::systemStartTime;
  for (std::size_t i=0; i<cloudInOut->size(); i++) {
    cloudInOut->points[i].intensity = curTime;
  }
}


void FARUtil::CropPCLCloud(const PointCloudPtr& cloudIn,
                          const PointCloudPtr& cloudCropped,
                          const Point3D& centriod,
                          const float& range) 
{
  const std::size_t cloud_size = cloudIn->size();
  std::size_t idx = 0;
  cloudCropped->clear(), cloudCropped->resize(cloud_size);
  for (const auto& p : cloudIn->points) {
    if ((centriod - p).norm() < range) {
      cloudCropped->points[idx] = p;
      idx ++;
    }
  }
  cloudCropped->resize(idx);
}

void FARUtil::CropPCLCloud(const PointCloudPtr& cloudInOut,
                          const Point3D& centriod,
                          const float& range) 
{
  PointCloudPtr temp_crop_ptr(new pcl::PointCloud<PCLPoint>());
  FARUtil::CropPCLCloud(cloudInOut, temp_crop_ptr, centriod, range);
  *cloudInOut = *temp_crop_ptr;
}

void FARUtil::CropCloudWithinHeight(const PointCloudPtr& cloudInOut, const float& height) {
  pcl::PointCloud<PCLPoint> temp_cloud;
  temp_cloud.resize(cloudInOut->points.size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    if (!FARUtil::IsPointInToleratedHeight(p, height)) continue;
    temp_cloud.points[idx] = p;
    idx ++;
  }
  temp_cloud.points.resize(idx);
  *cloudInOut = temp_cloud;
}

void FARUtil::TransformPCLFrame(const std::string& from_frame_id,
                               const std::string& to_frame_id,
                               const tf::TransformListener* tf_listener,
                               const PointCloudPtr& cloudInOut) 
{
  if (cloudInOut->empty()) return;
  pcl::PointCloud<PCLPoint> aft_tf_cloud;
  tf::StampedTransform cloud_to_map_tf;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, ros::Time(0), ros::Duration(1.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, ros::Time(0), cloud_to_map_tf);
  } catch (tf::TransformException ex){
    throw ex;
    return;
  }
  pcl_ros::transformPointCloud(*cloudInOut, aft_tf_cloud, cloud_to_map_tf);
  *cloudInOut = aft_tf_cloud;
}

void FARUtil::TransformPoint3DFrame(const std::string& from_frame_id,
                                   const std::string& to_frame_id,
                                   const tf::TransformListener* tf_listener,
                                   Point3D& point)
{
  tf::Vector3 point_vec(point.x, point.y, point.z);
  tf::StampedTransform transform_tf_stamp;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, ros::Time(0), ros::Duration(1.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, ros::Time(0), transform_tf_stamp);
    point_vec = transform_tf_stamp * point_vec;
  } catch (tf::TransformException ex){
    ROS_ERROR("Tracking Point3D TF lookup: %s",ex.what());
    return;
  }
  point.x = point_vec.x();
  point.y = point_vec.y();
  point.z = point_vec.z();
}

bool FARUtil::IsSameFrameID(const std::string& cur_frame, const std::string& ref_frame) {
  std::string str1 = cur_frame;
  std::string str2 = ref_frame;
  if (cur_frame[0] == '/') str1 = cur_frame.substr(1);
  if (ref_frame[0] == '/') str2 = ref_frame.substr(1);
  return str1 == str2;
}

void FARUtil::ConvertCloudToPCL(const PointStack& point_stack, 
                               const PointCloudPtr& point_cloud_ptr) 
{
  point_cloud_ptr->clear();
  point_cloud_ptr->points.resize(point_stack.size());
  std::size_t i = 0;
  for (const auto& p : point_stack) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.x)) {
      if (FARUtil::IsDebug) ROS_WARN_ONCE("FARUtil: nan or inf point detected.");
      continue;
    }
    PCLPoint point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_ptr->points[i] = point;
    i++;
  }
}

geometry_msgs::Point FARUtil::Point3DToGeoMsgPoint(const Point3D& point) {
  geometry_msgs::Point p;
  p.x = point.x;
  p.y = point.y;
  p.z = point.z;
  return p;
}

void FARUtil::ExtractFreeAndObsCloud(const PointCloudPtr& newCloudIn,
                                    const PointCloudPtr& freeCloudOut,
                                    const PointCloudPtr& obsCloudOut) 
{
  // pre-process cloud
  freeCloudOut->clear(), obsCloudOut->clear();
  const std::size_t cloud_size = newCloudIn->size();
  freeCloudOut->resize(cloud_size), obsCloudOut->resize(cloud_size);
  std::size_t free_idx = 0;
  std::size_t obs_idx = 0;
  // iteratte through points
  for (const auto& p : newCloudIn->points) {
    if (p.intensity < FARUtil::kFreeZ) {
      freeCloudOut->points[free_idx] = p;
      free_idx ++;
    } else {
      obsCloudOut->points[obs_idx] = p;
      obs_idx ++;
    }
  } 
  freeCloudOut->resize(free_idx), obsCloudOut->resize(obs_idx);
}

void FARUtil::UpdateKdTrees(const PointCloudPtr& newObsCloudIn) 
{
  if (!newObsCloudIn->empty()) {
    FARUtil::kdtree_new_cloud_->setInputCloud(newObsCloudIn);
  } else {
    FARUtil::ClearKdTree(newObsCloudIn, FARUtil::kdtree_new_cloud_);
  }
}

void FARUtil::ClearKdTree(const PointCloudPtr& cloud_ptr,
                          const PointKdTreePtr& kdTree_ptr) {
  PCLPoint temp_p;
  temp_p.x = temp_p.y = temp_p.z = 0.0f;
  cloud_ptr->resize(1), cloud_ptr->points[0] = temp_p;
  kdTree_ptr->setInputCloud(cloud_ptr);
}

bool FARUtil::IsPointNearNewPoints(const Point3D& p, const bool& is_creation) {
  const std::size_t near_c = FARUtil::PointInNewCounter(p, FARUtil::kMatchDist);
  const int counter_limit = is_creation ? std::round(FARUtil::KNewPointC / 2.0f) : FARUtil::KNewPointC;
  return (near_c > counter_limit) ? true : false;
}

std::size_t FARUtil::PointInXCounter(const Point3D& p,
                                     const float& radius,
                                     const PointKdTreePtr& KdTree) 
{
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  PCLPoint pcl_p;
  pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = p.z;
  if (!std::isfinite(pcl_p.x) || !std::isfinite(pcl_p.y) || !std::isfinite(pcl_p.z)) {
    return 0;
  }
  KdTree->radiusSearch(pcl_p, radius, pointSearchInd, pointSearchSqDis);
  return pointSearchInd.size();
}

std::size_t FARUtil::PointInNewCounter(const Point3D& p, const float& radius) {
  return FARUtil::PointInXCounter(p, radius, FARUtil::kdtree_new_cloud_);
}

void FARUtil::Flat3DPointCloud(const PointCloudPtr& cloudIn, 
                              const PointCloudPtr& cloudFlat,
                              const bool& is_downsample) 
{
  cloudFlat->clear();
  pcl::copyPointCloud(*cloudIn, *cloudFlat);
  const std::size_t cloud_size = cloudFlat->size();
  for (std::size_t i=0; i<cloud_size; i++) {
    cloudFlat->points[i].z = 0.0;
  }
  if (is_downsample) {
    FARUtil::FilterCloud(cloudFlat, FARUtil::kLeafSize);
  }
}

int FARUtil::Mod(const int& a, const int& b) {
  return (b + (a % b)) % b;
}

void FARUtil::EraseNodeFromStack(const NavNodePtr& node_ptr,
                                 NodePtrStack& node_stack) {
  for (auto it = node_stack.begin(); it != node_stack.end(); it++) {
    if (*it  == node_ptr) {
      node_stack.erase(it--);
    }
  }
}

bool FARUtil::IsSamePoint3D(const Point3D& p1, const Point3D& p2) {
  if ((p2 - p1).norm() < FARUtil::kEpsilon) {
    return true;
  }
  return false;
}

void FARUtil::SetDifference(std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& diff) {
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  diff.clear();
  std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(diff, diff.begin()));
}

float FARUtil::MarginAngleNoise(const float& dist, const float& max_shift_dist, const float& angle_noise) {
  float margin_angle_noise = angle_noise;
  if (dist * sin(margin_angle_noise) < max_shift_dist) {
    margin_angle_noise = std::asin(max_shift_dist / std::max(dist, max_shift_dist));
  }
  return margin_angle_noise;
}

bool FARUtil::IsOutReducedDirs(const Point3D& diff_p,
                               const PointPair& surf_dirs) 
{
  const Point3D norm_dir = diff_p.normalize_flat();
  const float margin_angle_noise = FARUtil::MarginAngleNoise(diff_p.norm_flat(), 
                                                             FARUtil::kNearDist, 
                                                             FARUtil::kAngleNoise);
  Point3D temp_opposite_dir;
  // check first half range
  temp_opposite_dir = -surf_dirs.second;
  float in_res_thred = FARUtil::NoiseCosValue(surf_dirs.first * temp_opposite_dir, true, margin_angle_noise);
  if (norm_dir * surf_dirs.first > in_res_thred &&
      norm_dir * temp_opposite_dir > in_res_thred) {
    return true;
  }
  // check second half range
  temp_opposite_dir = -surf_dirs.first;
  in_res_thred = FARUtil::NoiseCosValue(surf_dirs.second * temp_opposite_dir, true, margin_angle_noise);
  if (norm_dir * surf_dirs.second > in_res_thred &&
      norm_dir * temp_opposite_dir > in_res_thred) {
    return true;
  }
  return false;
}

bool FARUtil::IsOutReducedDirs(const Point3D& diff_p,
                               const NavNodePtr& node_ptr)
{
  if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
    if (!FARUtil::IsOutReducedDirs(diff_p, node_ptr->surf_dirs)) {
      return false;
    }
  }
  return true;
}

bool FARUtil::IsInCoverageDirPairs(const Point3D& diff_p, const NavNodePtr& node_ptr) {
  if (node_ptr->free_direct == NodeFreeDirect::PILLAR) return false;
  const Point3D norm_dir = diff_p.normalize_flat();
  const PointPair surf_dirs = node_ptr->surf_dirs;
  const float margin_angle_noise = FARUtil::MarginAngleNoise(diff_p.norm_flat(), 
                                                             FARUtil::kNearDist, 
                                                             FARUtil::kAngleNoise * 2.0f);
  const float dot_value = FARUtil::NoiseCosValue(surf_dirs.first * surf_dirs.second, true, margin_angle_noise);
  if (node_ptr->free_direct == NodeFreeDirect::CONCAVE) {
    if (norm_dir * surf_dirs.first > dot_value && norm_dir * surf_dirs.second > dot_value) {
      return true;
    }
  } else if (node_ptr->free_direct == NodeFreeDirect::CONVEX) {
    if (norm_dir * (-surf_dirs.second) > dot_value && norm_dir * (-surf_dirs.first) > dot_value) {
      return true;
    }
  }
  return false;
}

bool FARUtil::IsInContourDirPairs(const Point3D& diff_p,
                                 const PointPair& surf_dirs) 
{
  const float margin_angle_noise = FARUtil::MarginAngleNoise(diff_p.norm_flat(), 
                                                             FARUtil::kNearDist, 
                                                             FARUtil::kAngleNoise);
  const float margin_cos_value = cos(margin_angle_noise);
  // check first half range
  if (surf_dirs.first.norm_dot(diff_p) > margin_cos_value) {
    return true;
  }
  // check second half range
  if (surf_dirs.second.norm_dot(diff_p) > margin_cos_value) {
    return true;
  }
  return false;
}

float FARUtil::DirsDistance(const PointPair& ref_dirs, const PointPair& compare_dirs) {
  const float a_d1 = std::acos(ref_dirs.first.norm_dot(compare_dirs.first));
  const float a_d2 = std::acos(ref_dirs.second.norm_dot(compare_dirs.second));
  return a_d1 + a_d2;
}

Point3D FARUtil::SurfTopoDirect(const PointPair& dirs, bool& _is_wall) {
  const Point3D topo_dir = dirs.first + dirs.second;
  _is_wall = false;
  if (topo_dir.norm_flat() > FARUtil::kEpsilon) {
    return topo_dir.normalize_flat();
  } else {
    _is_wall = true;
    return Point3D(0,0,0);
  }
}

Point3D FARUtil::SurfTopoDirect(const PointPair& dirs) {
  bool UNUSE_iswall;
  return FARUtil::SurfTopoDirect(dirs, UNUSE_iswall);
}

void FARUtil::InflateCloud(const PointCloudPtr& obsCloudInOut, 
                           const float& resol,
                           const int& inflate_size,
                           const bool& deep_z_inflate) 
{
  const std::size_t current_size = obsCloudInOut->size();
  const int z_size = deep_z_inflate ? inflate_size + 1 : inflate_size;
  const std::size_t array_size = current_size * (pow(inflate_size*2+1, 2)*(z_size*2+1) + 1);
  obsCloudInOut->resize(array_size);
  std::size_t cur_idx = current_size;
  for (std::size_t p_idx=0; p_idx<current_size; p_idx++) {
    PCLPoint p = obsCloudInOut->points[p_idx];
    for (int ix=-inflate_size; ix<=inflate_size; ix++) {
      for (int iy=-inflate_size; iy<=inflate_size; iy++) {
        for (int iz=-z_size; iz<=z_size; iz++) {
          PCLPoint ref_p;
          ref_p.x = p.x + ix * resol;
          ref_p.y = p.y + iy * resol;
          ref_p.z = p.z + iz * resol;
          ref_p.intensity = p.intensity;
          obsCloudInOut->points[cur_idx] = ref_p;
          cur_idx ++;
        }
      }
    }
  }
  obsCloudInOut->resize(cur_idx);
  FARUtil::FilterCloud(obsCloudInOut, resol);
}

float FARUtil::NoiseCosValue(const float& dot_value, const bool& is_large, const float& noise) {
  const float crop_value = FARUtil::ClampAbsRange(dot_value, 1.0f);
  const float theta = std::acos(dot_value);
  const int sign = is_large ? 1 : -1;
  double margin_theta = theta + sign * noise;
  margin_theta = std::min(std::max(margin_theta, 0.0), M_PI);
  return cos(margin_theta);
}

void FARUtil::ClusterFilterCloud(const PointCloudPtr& cloudInOut,
                                const float& radius,
                                const std::size_t c_thred) {
  PointCloudPtr temp_ptr(new pcl::PointCloud<PCLPoint>());
  if (cloudInOut->empty()) return;
  FARUtil::kdtree_filter_cloud_->setInputCloud(cloudInOut);
  temp_ptr->clear(), temp_ptr->resize(cloudInOut->size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    const std::size_t counter = FARUtil::PointInXCounter(Point3D(p), radius, FARUtil::kdtree_filter_cloud_);
    if (counter > c_thred) {
      temp_ptr->points[idx] = p;
      idx ++;
    }
  }
  temp_ptr->resize(idx), *cloudInOut = *temp_ptr;
  return;
}

void FARUtil::TransferCloud(const Point3D& transPoint, const PointCloudPtr& cloudInOut) {
  PointCloudPtr temp_ptr(new pcl::PointCloud<PCLPoint>());
  pcl::copyPointCloud(*cloudInOut, *temp_ptr);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0,3) = transPoint.x;
  transform(1,3) = transPoint.y;
  transform(2,3) = transPoint.z;
  pcl::transformPointCloud(*temp_ptr, *cloudInOut, transform);
}

void FARUtil::ExtractOverlapCloud(const PointCloudPtr& cloudIn,
                                 const PointCloudPtr& cloudRef,
                                 const PointCloudPtr& cloudOverlapOut,
                                 const float& margin_ratio)
{
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  FARUtil::ResetCloudIntensity(cloudIn, true);
  FARUtil::ResetCloudIntensity(cloudRef, false);
  *temp_cloud = *cloudIn + *cloudRef;
  const float leaf_size = margin_ratio * FARUtil::kLeafSize;;
  FARUtil::FilterCloud(temp_cloud, leaf_size);
  cloudOverlapOut->clear(), cloudOverlapOut->resize(temp_cloud->size());
  std::size_t idx = 0;
  for (const auto& p : temp_cloud->points) {
    if (p.intensity > 0.0 && p.intensity < 255.0) {
      cloudOverlapOut->points[idx] = p;
      idx ++;
    }
  }
  cloudOverlapOut->resize(idx);
}

void FARUtil::RemoveOverlapCloud(const PointCloudPtr& cloudInOut,
                                 const PointCloudPtr& cloudRef,
                                 const bool& is_copy_cloud) 
{
  if (cloudRef->empty() || cloudInOut->empty()) return;
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  PointCloudPtr ref_cloud = cloudRef;
  if (is_copy_cloud) {
    PointCloudPtr copyRefCloud(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*cloudRef, *copyRefCloud);
    ref_cloud = copyRefCloud;
  }
  FARUtil::ResetCloudIntensity(cloudInOut, true);
  FARUtil::ResetCloudIntensity(ref_cloud, false);
  *temp_cloud = *cloudInOut + *ref_cloud;
  const float leaf_size = FARUtil::kLeafSize * 1.2;
  FARUtil::FilterCloud(temp_cloud, leaf_size);
  cloudInOut->clear(), cloudInOut->resize(temp_cloud->size());
  std::size_t idx = 0;
  for (const auto& p : temp_cloud->points) {
    if (p.intensity < 255.0) continue;
    cloudInOut->points[idx] = p;
    idx++;
  }
  cloudInOut->resize(idx);
}

void FARUtil::StackCloudByTime(const PointCloudPtr& curInCloud,
                              const PointCloudPtr& StackCloud,
                              const float& duration) 
{
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  outliers->indices.resize(StackCloud->size());
  FARUtil::ResetCloudIntensityWithTime(curInCloud);
  *StackCloud += *curInCloud;
  std::size_t idx = 0;
  std::size_t outIdx = 0;
  const float curTime = ros::Time::now().toSec() - FARUtil::systemStartTime;
  for (const auto& pcl_p : StackCloud->points) {
    if (curTime - pcl_p.intensity > duration) {
      outliers->indices[outIdx] = idx;
      outIdx ++;
    }
    idx ++;
  }
  outliers->indices.resize(outIdx);
  FARUtil::RemoveIndicesFromCloud(outliers, StackCloud);
}

void FARUtil::RemoveIndicesFromCloud(const pcl::PointIndices::Ptr& outliers,
                                    const PointCloudPtr& cloudInOut) {
  pcl::ExtractIndices<PCLPoint> extract;
  extract.setInputCloud(cloudInOut);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*cloudInOut);
}

float FARUtil::ApproxAtan2(const float& y, const float x) {
  const float ay = std::abs(y);
  const float ax = std::abs(x);
  const float n1 = 0.97239411f;
  const float n2 = -0.19194795f;
  const float z = ay > ax ? ax / ay : ay / ax;  // [0,1]
  float th = (n1 + n2 * z * z) * z;
  if (ay > ax)
    th = M_PI_2 - th;  // [0,π/2]
  if (x < 0)
    th = M_PI - th;      // [0,π]
  th = copysign(th, y);  // [-π,π]
  return th;
}

float FARUtil::PixelDistance(const cv::Point2f& pre_p, const cv::Point2f& cur_p) {
  return std::hypotf(pre_p.x - cur_p.x, pre_p.y - cur_p.y);
}

float FARUtil::VerticalDistToLine2D(const Point3D& start_p, 
                                  const Point3D& end_p, 
                                  const Point3D& cur_p) 
{
  const Point3D line_dir = end_p - start_p;
  const Point3D diff_p   = cur_p - start_p;
  const float dot_value  = line_dir.norm_flat_dot(diff_p);
  return sin(acos(dot_value)) * diff_p.norm_flat();
}

Point3D FARUtil::ContourSurfDirs(const Point3D& end_p, 
                                 const Point3D& start_p, 
                                 const Point3D& center_p,
                                 const float& radius) 
{
  const float D = (center_p - end_p).norm_flat();
  const float phi = std::acos((center_p - end_p).norm_flat_dot(start_p - end_p));
  const float H = D * sin(phi);
  if (H < FARUtil::kEpsilon) { // co-linear
    return (end_p - center_p).normalize_flat();
  }
  const float theta = asin(FARUtil::ClampAbsRange(H / radius, 1.0f));
  const Point3D dir = (start_p - end_p).normalize_flat();
  const Point3D V_p = end_p + dir * D * cos(phi);
  const Point3D K_p = V_p - dir * radius * cos(theta);
  return (K_p - center_p).normalize_flat();;
}

float FARUtil::CosinTheta(const Point3D& vertex, const Point3D& p1, const Point3D& p2) {
  const Point3D dir1 = p1 - vertex;
  const Point3D dir2 = p2 - vertex;
  return dir1.norm_dot(dir2);
}

void FARUtil::CorrectDirectOrder(const PointPair& ref_dir, PointPair& dirInOUt) {
  PointPair temp_dir = dirInOUt;
  if (ref_dir.first * dirInOUt.first  + ref_dir.second * dirInOUt.second < 
      ref_dir.first * dirInOUt.second + ref_dir.second * dirInOUt.first) 
  {
      dirInOUt.first = temp_dir.second;
      dirInOUt.second = temp_dir.first;
  }
} 

std::size_t FARUtil::CounterOfPillar(const std::deque<PointPair>& dirs_stack) {
  std::size_t counter = 0;
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  for (const auto& p_pair : dirs_stack) {
    if (p_pair == pillar_dir) {
      counter ++;
    }
  }
  return counter;
}

Point3D FARUtil::RANSACPoisiton(const std::deque<Point3D>& pos_filter_stack, const float& margin, std::size_t& inlier_size) {
  inlier_size = 0;
  PointStack inlier_stack;
  for (const auto& p : pos_filter_stack) {
    std::size_t temp_inlier_size = 0;
    PointStack temp_inlier_stack;
    temp_inlier_stack.clear();
    for (const auto& cp : pos_filter_stack) {
      if ((p - cp).norm_flat() < margin) {
        temp_inlier_stack.push_back(cp);
        temp_inlier_size ++;
      }
    }
    if (temp_inlier_size > inlier_size) {
      inlier_stack = temp_inlier_stack;
      inlier_size = temp_inlier_size;
    }
  }
  return FARUtil::AveragePoints(inlier_stack);
}

PointPair FARUtil::RANSACSurfDirs(const std::deque<PointPair>& surf_dirs_stack, const float& margin, std::size_t& inlier_size) {
  inlier_size = 0;
  std::vector<PointPair> inlier_stack;
  const std::size_t pillar_size = FARUtil::CounterOfPillar(surf_dirs_stack);
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  for (const auto& dir_pair : surf_dirs_stack) {
    if (dir_pair == pillar_dir) continue;
    std::size_t temp_inlier_size = 0;
    std::vector<PointPair> temp_inlier_stack;
    temp_inlier_stack.clear();
    for (const auto& cdir_pair : surf_dirs_stack) {
      if (cdir_pair == pillar_dir) continue;
      if (FARUtil::DirsDistance(dir_pair, cdir_pair) < margin) {
        temp_inlier_stack.push_back(cdir_pair);
        temp_inlier_size ++;
      }
    }
    if (temp_inlier_size > inlier_size) {
      inlier_stack = temp_inlier_stack;
      inlier_size = temp_inlier_size;
    }
  }
  if (pillar_size > inlier_size) { // this node should be a pillar
    inlier_size = pillar_size;
    return pillar_dir;
  }
  return FARUtil::AverageDirs(inlier_stack);
}

PointPair FARUtil::AverageDirs(const std::vector<PointPair>& dirs_stack) {
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  Point3D mean_dir1(0,0,0);
  Point3D mean_dir2(0,0,0);
  for (const auto& dir_pair : dirs_stack) {
    if (dir_pair == pillar_dir) continue;
    mean_dir1 = mean_dir1 + dir_pair.first;
    mean_dir2 = mean_dir2 + dir_pair.second; 
  }
  mean_dir1 = mean_dir1.normalize();
  mean_dir2 = mean_dir2.normalize();
  return PointPair(mean_dir1, mean_dir2);
}

void FARUtil::ConvertCTNodeStackToPCL(const CTNodeStack& ctnode_stack, 
                                     const PointCloudPtr& point_cloud_ptr)
{
  const std::size_t N = ctnode_stack.size();
  point_cloud_ptr->clear(), point_cloud_ptr->resize(N);
  for (std::size_t i=0; i<N; i++) {
    point_cloud_ptr->points[i] = FARUtil::Point3DToPCLPoint(ctnode_stack[i]->position);
  }
}

void FARUtil::CropBoxCloud(const PointCloudPtr& cloudInOut, 
                          const Point3D& center_p, 
                          const Point3D& crop_size) 
{
  pcl::CropBox<PCLPoint> boxFilter;
  pcl::PointCloud<PCLPoint> cropBox_cloud;
  Eigen::Vector4f min_vec(center_p.x - crop_size.x,
                          center_p.y - crop_size.y,
                          center_p.z - crop_size.z, 1.0f);
  Eigen::Vector4f max_vec(center_p.x + crop_size.x,
                          center_p.y + crop_size.y,
                          center_p.z + crop_size.z, 1.0f);
  boxFilter.setMin(min_vec), boxFilter.setMax(max_vec);
  boxFilter.setInputCloud(cloudInOut);
  boxFilter.filter(cropBox_cloud);
  *cloudInOut = cropBox_cloud;
}

bool FARUtil::IsInCylinder(const Point3D& from_p, const Point3D& end_p, const Point3D& cur_p, const float& radius, const bool& is_2D) {
  const Point3D unit_axial = is_2D ? (end_p - from_p).normalize_flat() : (end_p - from_p).normalize();
  const Point3D vec = cur_p - from_p;
  const float proj_scalar = vec * unit_axial;
  float temp_line_dist = is_2D ? (end_p - from_p).norm_flat() : (end_p - from_p).norm();
  if (proj_scalar < - radius || proj_scalar > temp_line_dist + radius) {
      return false;
  }
  const Point3D vec_axial = unit_axial * proj_scalar;
  temp_line_dist = is_2D ? (vec - vec_axial).norm_flat() : (vec - vec_axial).norm();
  if (temp_line_dist > radius) {
      return false;
  }
  return true;
}

void FARUtil::CreatePointsAroundCenter(const Point3D& center_p, 
                                      const float& radius,
                                      const float& resol,
                                      PointStack& points_stack,
                                      const bool& is_sort)
{
  const int H_SIZE = std::ceil(radius / resol);
  const std::size_t grid_size = (2*H_SIZE+1) * (2*H_SIZE+1);
  points_stack.clear(), points_stack.resize(grid_size);
  std::size_t idx = 0;
  for (int i=-H_SIZE; i<=H_SIZE; i++) {
      for (int j=-H_SIZE; j<=H_SIZE; j++) {
          Point3D p(center_p.x+resol*i, center_p.y+resol*j, center_p.z);
          p.intensity = (p - center_p).norm();
          points_stack[idx] = p;
          idx ++;
      }
  }
  points_stack.resize(idx);
  if (is_sort) {
    std::sort(points_stack.begin(), points_stack.end(), intensity_comp());
  }
}

bool FARUtil::IsVoteTrue(const std::deque<int>& votes, const bool& is_balance) {
  const int N = votes.size();
  const float sum = std::accumulate(votes.begin(), votes.end(), 0);
  const float factor = is_balance ? 2.0f : 3.0f;
  if (sum > std::floor(N / factor)) {
      return true;
  }
  return false;
}

int FARUtil::VoteRankInVotes(const int& c, const std::vector<int>& ordered_votes) {
  int idx = 0;
  while (idx < ordered_votes.size() && c < ordered_votes[idx]) {
      idx ++;
  }
  return idx;
}

bool FARUtil::IsOutReachNode(const NavNodePtr& node_ptr) {
  // check whether or not this node is connect to a node beyond planning range
  for (const auto& cnode : node_ptr->connect_nodes) {
      if (!cnode->is_near_nodes && !FARUtil::IsOutsideGoal(cnode) &&
          !FARUtil::IsTypeInStack(cnode, node_ptr->contour_connects)) 
      {
          return true;
      }
  }
  return false;
}

bool FARUtil::IsPointInLocalRange(const Point3D& p, const bool& is_large_h) {
  const float H = is_large_h ? FARUtil::kTolerZ + FARUtil::kHeightVoxel : FARUtil::kTolerZ;
  if (FARUtil::IsPointInToleratedHeight(p, H) && (p - FARUtil::odom_pos).norm() < FARUtil::kSensorRange) {
      return true;
  }
  return false;
}

bool FARUtil::IsPointInMarginRange(const Point3D& p) {
  if (FARUtil::IsPointInToleratedHeight(p, FARUtil::kMarginHeight) && (p - FARUtil::odom_pos).norm() < FARUtil::kMarginDist) {
      return true;
  }
  return false;
}

float FARUtil::DistanceToLineSeg2D(const Point3D& p, const PointPair& line) {
  /* Source: https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment */
  float A, B, C, D, dot, len_sq, param, xx, yy, dx, dy;
  A = (p - line.first).x, B = (p - line.first).y;
  C = (line.second - line.first).x, D = (line.second - line.first).y;
  dot = A * C + B * D;
  len_sq = C * C + D * D;
  param = -1.0f;
  if (len_sq != 0.0f) param = dot / len_sq;
  if (param < 0.0f) {
    xx = line.first.x, yy = line.first.y;
  } else if (param > 1.0f) {
    xx = line.second.x, yy = line.second.y;
  } else {
    xx = line.first.x + param * C;
    yy = line.first.y + param * D;
  }
  dx = p.x - xx, dy = p.y - yy;
  return sqrt(dx * dx + dy * dy);
}

bool FARUtil::ClockwiseLess(const Point3D& a, const Point3D& b) {
  /* Source: https://stackoverflow.com/questions/6989100/sort-points-in-clockwise-order */
  if (a.x >= 0.0f && b.x < 0.0f)
        return true;
    if (a.x < 0.0f && b.x >= 0.0f)
        return false;
    if (a.x == 0.0f && b.x == 0.0f) {
        if (a.y >= 0.0f || b.y >= 0.0f) return a.y > b.y;
        return b.y > a.y;
    }
    // compute the cross product of vectors (center -> a) x (center -> b)
    float det = (a.x ) * (b.y) - (b.x ) * (a.y);
    if (det < 0.0f)
        return true;
    if (det > 0.0f)
        return false;
    // points a and b are on the same line from the center
    // check which point is closer to the center
    float d1 = (a.x) * (a.x) + (a.y) * (a.y);
    float d2 = (b.x) * (b.x) + (b.y) * (b.y);
    return d1 > d2;
}

void FARUtil::ClockwiseTwoPoints(const Point3D& center, PointPair& edge) {
  PointPair edge_copy;
  edge_copy.first = Point3D(edge.first), edge_copy.second = Point3D(edge.second);
  Point3D center_copy = Point3D(center);
  const Point3D c = (center + edge.first + edge.second) / 3.0f;
  PointStack vertices;
  edge_copy.first = edge_copy.first - c; vertices.push_back(edge_copy.first);
  edge_copy.second = edge_copy.second - c; vertices.push_back(edge_copy.second);
  center_copy = center_copy - c; vertices.push_back(center_copy);
  std::sort(vertices.begin(), vertices.end(), ClockwiseLess);
  int center_id = 0;
  for (int i=0; i<3; i++) {
    if (FARUtil::IsSamePoint3D(vertices[i], center_copy)) {
      center_id = i;
      break;
    }
  }
  edge.first = vertices[FARUtil::Mod(center_id+1, 3)] + c;
  edge.second = vertices[FARUtil::Mod(center_id+2, 3)] + c;
}

struct {
  bool operator() (const PointPair& edge1, const PointPair& edge2) const {
    const Point3D center1 = (edge1.first + edge1.second) / 2.0f;
    const Point3D center2 = (edge2.first + edge2.second) / 2.0f;
    return FARUtil::ClockwiseLess(center1, center1); 
  }
} EdgeClockwiseLess;

void FARUtil::SortEdgesClockWise(const Point3D& center, std::vector<PointPair>& edges) {
  for (auto& edge : edges) {
    edge.first  = edge.first - center;
    edge.second = edge.second - center; 
  }
  std::sort(edges.begin(), edges.end(), EdgeClockwiseLess);
  for (auto& edge : edges) {
    edge.first  = edge.first + center;
    edge.second = edge.second + center; 
  }
  for (auto& edge : edges) {
    FARUtil::ClockwiseTwoPoints(center, edge);
  }
}

float FARUtil::LineMatchPercentage(const PointPair& line1, const PointPair& line2) {
  const float ds = (line1.first - line2.first).norm_flat();
  const float theta = acos((line1.second - line1.first).norm_flat_dot(line2.second - line2.first));
  if (theta > FARUtil::kAcceptAlign || ds > FARUtil::kNavClearDist) return 0.0f;
  const float contour_ds = (line2.second - line2.first).norm_flat();
  float match_ds = contour_ds;
  if (theta > FARUtil::kEpsilon) {
    match_ds = std::min(match_ds, FARUtil::kNavClearDist / tan(theta));
  }
  return match_ds / contour_ds;
}