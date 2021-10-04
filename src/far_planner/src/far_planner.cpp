/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/far_planner.h"

/***************************************************************************************/

void FARMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_    = nh.subscribe("/reset_visibility_graph", 5, &FARMaster::ResetGraphCallBack, this);
  odom_sub_           = nh.subscribe("/odom_world", 5, &FARMaster::OdomCallBack, this);
  terrain_sub_        = nh.subscribe("/terrain_cloud", 1, &FARMaster::TerrainCallBack, this);
  scan_sub_           = nh.subscribe("/scan_cloud", 5, &FARMaster::ScanCallBack, this);
  waypoint_sub_       = nh.subscribe("/goal_point", 1, &FARMaster::WaypointCallBack, this);
  terrain_local_sub_  = nh.subscribe("/terrain_local_cloud", 1, &FARMaster::TerrainLocalCallBack, this);
  joy_command_sub_    = nh.subscribe("/joy", 5, &FARMaster::JoyCommandCallBack, this);
  update_command_sub_ = nh.subscribe("/update_visibility_graph", 5, &FARMaster::UpdateCommandCallBack, this);
  goal_pub_           = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
  runtime_pub_        = nh.advertise<std_msgs::Float32>("/runtime",1);

  //DEBUG Publisher
  dynamic_obs_pub_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_dynamic_obs_debug",1);
  surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug",1);
  surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug",1);
  scan_grid_debug_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_scanGrid_debug",1);
  new_PCL_pub_         = nh.advertise<sensor_msgs::PointCloud2>("/FAR_new_debug",1);

  this->LoadROSParams();

  /*init path generation thred callback*/
  const float duration_time = 0.99f / master_params_.main_run_freq;
  planning_event_ = nh.createTimer(ros::Duration(duration_time), &FARMaster::PlanningCallBack, this);

  /* init Dynamic Planner Processing Objects */
  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh, graph_params_);
  graph_planner_.Init(nh, gp_params_);
  contour_graph_.Init(cg_params_);
  planner_viz_.Init(nh);
  map_handler_.Init(map_params_);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh, msger_parmas_);

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_graph_init_      = false;
  is_reset_env_       = false;
  is_stop_update_     = false;

  // allocate memory to pointers
  new_vertices_ptr_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_terrain_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());

  // init global utility cloud
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();

  // init TF listener
  tf_listener_ = new tf::TransformListener();

  // clear temp vectors and memory
  this->ClearTempMemory();
  FARUtil::robot_pos = Point3D(0,0,0);
  FARUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  nav_goal_    = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
  std::cout<<std::endl;
  if (master_params_.is_static_env) {
    std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
  } else {
    std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
  }
  std::cout<<"\n"<<std::endl;
}

void FARMaster::ResetEnvironmentAndGraph() {
  this->ResetInternalValues();
  if (!FARUtil::IsDebug) { // Terminal Output
    printf("\033[A"), printf("\033[A"), printf("\033[2K");
    std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
  }
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  /* Reset clouds */
  FARUtil::surround_obs_cloud_->clear();
  FARUtil::surround_free_cloud_->clear();
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();
  FARUtil::cur_new_cloud_->clear();
  FARUtil::cur_dyobs_cloud_->clear();
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = ros::Time::now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_.publish(goal_waypoint_stamped_);
  PointStack empty_path;
  planner_viz_.VizPath(empty_path);
}

void FARMaster::Loop() {
  ros::Rate loop_rate(master_params_.main_run_freq);
  while (ros::ok()) {
    if (is_reset_env_) {
      this->ResetEnvironmentAndGraph(); 
      is_reset_env_ = false;
      if (FARUtil::IsDebug) ROS_WARN("****************** Graph and Env Reset ******************");
      loop_rate.sleep(); // skip this iteration
      continue;
    }
    /* Process callback functions */
    ros::spinOnce(); 
    if (!this->PreconditionCheck()) {
      loop_rate.sleep();
      continue;
    }
    /* add main process after this line */
    graph_manager_.UpdateOdom(robot_pos_);
    odom_node_ptr_ = graph_manager_.GetOdomNode();
    if (odom_node_ptr_ == NULL) {
      ROS_WARN("FAR: Waiting for Odometry...");
      loop_rate.sleep();
      continue;
    }
    this->ClearTempMemory();
    map_handler_.UpdateRobotPosition(odom_node_ptr_->position);
    /* Extract Vertices and new nodes */
    FARUtil::Timer.start_time("Total V-Graph Update");
    contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
    contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Local V-Graph Updated. Number of local vertices: "<<ContourGraph::contour_graph_.size()<<std::endl;
    }
    near_nav_graph_ = graph_manager_.GetNearNavGraph();
    contour_graph_.MatchContourWithNavGraph(near_nav_graph_, new_ctnodes_);
    if (master_params_.is_visual_opencv) {
      FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
      cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
      contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
    }

    /* update planner graph */
    if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
      new_nodes_ = graph_manager_.GetNewNodes();
    }
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<< "Number of new vertices adding to global V-Graph: "<< new_nodes_.size()<<std::endl;
    }
    /* Graph Updating */
    graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);

    runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
    runtime_pub_.publish(runtimer_);

    /* Update v-graph in other modules */
    nav_graph_ = graph_manager_.GetNavGraph();
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Global V-Graph Updated. Number of global vertices: "<<nav_graph_.size()<<std::endl;
    }
    contour_graph_.ExtractGlobalContours(nav_graph_);
    /* Planner Graph Update */
    graph_planner_.UpdaetVGraph(nav_graph_);
    /* Graph Messager Update */
    graph_msger_.UpdateGlobalGraph(nav_graph_);

    /* Viz Navigation Graph */
    const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
    if (last_internav_ptr != NULL) {
      planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
    }
    planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
    planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
    planner_viz_.VizGraph(nav_graph_);
    planner_viz_.VizContourGraph(ContourGraph::contour_graph_);

    if (is_graph_init_) { 
      if (FARUtil::IsDebug) {
        std::cout<<" ========================================================== "<<std::endl;
      } else { // cleanup outputs in terminal
        for (int i = 0; i < 6; i++) {
          printf("\033[A");
        }
      }
    }

    if (!is_graph_init_ && !ContourGraph::global_contour_.empty()) {
      is_graph_init_ = true;
      printf("\033[A"), printf("\033[A"), printf("\033[2K");
      std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
    }
    loop_rate.sleep();
  }
}

void FARMaster::PlanningCallBack(const ros::TimerEvent& event) {
  if (!is_graph_init_) return;
  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  if (goal_ptr == NULL) {
    /* Graph Traversablity Update */
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Adding Goal to V-Graph "<<"Time: "<<0.f<<"ms"<<std::endl;
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Path Search "<<"Time: "<<0.f<<"ms"<<std::endl;
  } else { 
    // Update goal postion with nearby terrain cloud
    const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
    PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
    PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
    map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
    map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
    graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
    graph_planner_.ReEvaluateGoalPosition(goal_ptr);

    // Adding goal into v-graph
    FARUtil::Timer.start_time("Adding Goal to V-Graph");
    graph_planner_.UpdateGoalNavNodeConnects(goal_ptr); 
    graph_planner_.UpdaetVGraph(graph_manager_.GetNavGraph());
    if (!FARUtil::IsDebug) printf("\033[2K");
    FARUtil::Timer.end_time("Adding Goal to V-Graph");

    // Update v-graph traversibility 
    FARUtil::Timer.start_time("Path Search");
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);

    // Construct path to gaol and publish waypoint
    PointStack global_path;
    Point3D last_nav_goal, current_free_goal;
    bool is_planning_fails = false;
    last_nav_goal = nav_goal_;
    goal_waypoint_stamped_.header.stamp = ros::Time::now();
    bool is_current_free_nav = false;
    if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_goal_, current_free_goal, is_planning_fails, is_current_free_nav)) {
      Point3D waypoint = nav_goal_;
      if ((waypoint - current_free_goal).norm() > FARUtil::kEpsilon) {
        waypoint = this->ProjectNavWaypoint(waypoint, last_nav_goal);
      }
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
      goal_pub_.publish(goal_waypoint_stamped_);
      is_planner_running_ = true;
      planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
      planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
      planner_viz_.VizPath(global_path, is_current_free_nav);
    } else if (is_planner_running_) {
      // stop robot
      global_path.clear();
      planner_viz_.VizPath(global_path);
      is_planner_running_ = false;
      nav_heading_ = Point3D(0,0,0);
      if (is_planning_fails) { // stops the robot
        goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
        goal_pub_.publish(goal_waypoint_stamped_);
      }
    }
    if (!FARUtil::IsDebug) printf("\033[2K");
    FARUtil::Timer.end_time("Path Search");
  }
}


Point3D FARMaster::ProjectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint) {
  const bool is_momentum = (last_waypoint - nav_waypoint).norm() < FARUtil::kEpsilon ? true : false; // momentum heading if same goal
  Point3D waypoint = nav_waypoint;
  const float dist = master_params_.waypoint_project_dist;
  const Point3D diff_p = nav_waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
    const float hdist = dist / 2.0f;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0f) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < dist) {
    waypoint = nav_waypoint + nav_heading_ * (dist - diff_p.norm());
  }
  return waypoint;
}

void FARMaster::LoadROSParams() {
  const std::string master_prefix   = "/far_planner/";
  const std::string map_prefix      = master_prefix + "MapHandler/";
  const std::string scan_prefix     = master_prefix + "ScanHandler/";
  const std::string cdetect_prefix  = master_prefix + "CDetector/";
  const std::string graph_prefix    = master_prefix + "Graph/";
  const std::string viz_prefix      = master_prefix + "Viz/";
  const std::string utility_prefix  = master_prefix + "Util/";
  const std::string planner_prefix  = master_prefix + "GPlanner/";
  const std::string contour_prefix  = master_prefix + "ContourGraph/";
  const std::string msger_prefix    = master_prefix + "GraphMsger/";

  // master params
  nh.param<float>(master_prefix + "main_run_freq",         master_params_.main_run_freq, 5.0);
  nh.param<float>(master_prefix + "voxel_dim",             master_params_.voxel_dim, 0.2);
  nh.param<float>(master_prefix + "robot_dim",             master_params_.robot_dim, 0.5);
  nh.param<float>(master_prefix + "vehicle_height",        master_params_.vehicle_height, 0.75);
  nh.param<float>(master_prefix + "sensor_range",          master_params_.sensor_range, 50.0);
  nh.param<float>(master_prefix + "terrain_range",         master_params_.terrain_range, 15.0);
  nh.param<float>(master_prefix + "waypoint_project_dist", master_params_.waypoint_project_dist, 5.0);
  nh.param<float>(master_prefix + "visualize_ratio",       master_params_.viz_ratio, 1.0);
  nh.param<bool>(master_prefix  + "is_opencv_visual",      master_params_.is_visual_opencv, true);
  nh.param<bool>(master_prefix  + "is_static_env",         master_params_.is_static_env, true);
  nh.param<bool>(master_prefix  + "is_debug_output",       master_params_.is_debug_output, false);
  nh.param<bool>(master_prefix  + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true);
  nh.param<std::string>(master_prefix + "world_frame",     master_params_.world_frame, "map");
  master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

  // map handler params
  nh.param<float>(map_prefix + "ceil_length",         map_params_.ceil_length, 5.0);
  nh.param<float>(map_prefix + "ceil_height",         map_params_.ceil_height, 2.0);
  nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0);
  nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0);
  map_params_.sensor_range = master_params_.sensor_range;

  // graph messager params
  nh.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
  msger_parmas_.frame_id = master_params_.world_frame;

  // graph planner params
  nh.param<float>(planner_prefix + "converge_distance",    gp_params_.converge_dist, 1.0);
  nh.param<float>(planner_prefix + "goal_adjust_radius",   gp_params_.adjust_radius, 10.0);
  nh.param<int>(planner_prefix   + "free_counter_thred",   gp_params_.free_thred, 5);
  nh.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 5);
  nh.param<float>(planner_prefix + "path_momentum_ratio",  gp_params_.momentum_dist, 1.0);
  nh.param<int>(planner_prefix   + "path_momentum_thred",  gp_params_.momentum_thred, 5);
  nh.param<int>(planner_prefix   + "clear_inflate_size",   gp_params_.clear_inflate_size, 3);
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // dynamic graph params
  nh.param<float>(graph_prefix  + "node_near_dist",            graph_params_.near_dist, 1.0);
  nh.param<float>(graph_prefix  + "match_margin_dist",         graph_params_.margin_dist, 1.5);
  nh.param<float>(graph_prefix  + "move_thred",                graph_params_.move_thred, 0.25);
  nh.param<int>(graph_prefix    + "connect_votes_size",        graph_params_.votes_size, 10);
  nh.param<float>(graph_prefix  + "trajectory_interval_ratio", graph_params_.traj_interval_ratio, 2.0);
  nh.param<int>(graph_prefix    + "terrain_inflate_size",      graph_params_.terrain_inflate, 2);
  nh.param<int>(graph_prefix    + "clear_dumper_thred",        graph_params_.dumper_thred, 3);
  nh.param<int>(graph_prefix    + "node_finalize_thred",       graph_params_.finalize_thred, 3);
  nh.param<int>(graph_prefix    + "filter_pool_size",          graph_params_.pool_size, 12);
  nh.param<float>(graph_prefix  + "connect_angle_thred",       graph_params_.kConnectAngleThred, 10);
  nh.param<float>(graph_prefix  + "pos_filter_margin",         graph_params_.filter_pos_margin, 0.5);
  nh.param<float>(graph_prefix  + "dirs_filter_margin",        graph_params_.filter_dirs_margin, 10.0);
  graph_params_.kConnectAngleThred = graph_params_.kConnectAngleThred / 180.0 * M_PI;
  graph_params_.filter_dirs_margin = graph_params_.filter_dirs_margin / 180.0 * M_PI;
  graph_params_.sensor_range       = master_params_.sensor_range;

  // utility params
  nh.param<float>(utility_prefix + "angle_noise",            FARUtil::kAngleNoise, 15.0);
  nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0);
  nh.param<float>(utility_prefix + "new_intensity_thred",    FARUtil::kNewPIThred, 2.0);
  nh.param<float>(utility_prefix + "nav_clear_dist",         FARUtil::kNavClearDist, 0.5);
  nh.param<float>(utility_prefix + "edge_project_dist",      FARUtil::kProjectDist, 0.4);
  nh.param<float>(utility_prefix + "terrain_free_Z",         FARUtil::kFreeZ, 0.1);
  nh.param<int>(utility_prefix   + "dyosb_update_thred",     FARUtil::kDyObsThred, 4);
  nh.param<int>(utility_prefix   + "new_point_counter",      FARUtil::KNewPointC, 10);
  nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
  nh.param<float>(utility_prefix + "new_points_decay_time",  FARUtil::kNewDecayTime, 2.0);
  FARUtil::worldFrameId   = master_params_.world_frame;
  FARUtil::kVizRatio      = master_params_.viz_ratio;
  FARUtil::kTolerZ        = map_params_.ceil_height * 1.5f;
  FARUtil::kCellLength    = map_params_.ceil_length;
  FARUtil::kAcceptAlign   = FARUtil::kAcceptAlign / 180.0 * M_PI;
  FARUtil::kAngleNoise    = FARUtil::kAngleNoise  / 180.0 * M_PI; 
  FARUtil::robot_dim      = master_params_.robot_dim;
  FARUtil::IsStaticEnv    = master_params_.is_static_env;
  FARUtil::IsDebug        = master_params_.is_debug_output;
  FARUtil::vehicle_height = master_params_.vehicle_height;
  FARUtil::kLeafSize      = master_params_.voxel_dim;
  FARUtil::kNearDist      = graph_params_.near_dist;
  FARUtil::kSensorRange   = master_params_.sensor_range;
  FARUtil::kTerrainRange  = master_params_.terrain_range;

  // scan handler params
  nh.param<int>(scan_prefix + "inflate_scan_size", scan_params_.inflate_size, 2);
  scan_params_.terrain_range = master_params_.terrain_range;
  scan_params_.voxel_size    = master_params_.voxel_dim;
  scan_params_.ceil_height   = FARUtil::kTolerZ * 2.0f;

  // contour detector params
  nh.param<float>(cdetect_prefix       + "resize_ratio",       cdetect_params_.kRatio, 5.0);
  nh.param<int>(cdetect_prefix         + "img_blur_size",      cdetect_params_.kBlurSize, 10);
  nh.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 6);
  nh.param<bool>(cdetect_prefix        + "is_save_img",        cdetect_params_.is_save_img, false);
  nh.param<std::string>(cdetect_prefix + "img_folder_path",    cdetect_params_.img_path, "");
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.voxel_dim    = master_params_.voxel_dim;

  // contour graph params
  nh.param<float>(contour_prefix + "around_distance",       cg_params_.kAroundDist, 0.75);
  nh.param<float>(contour_prefix + "pillar_perimeter_dist", cg_params_.kPillarPerimeter, 3.0);
  cg_params_.kMatchDist = graph_params_.margin_dist;
}

void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf::Pose tf_odom_pose;
  tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
  if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
    tf::StampedTransform odom_to_world_tf_stamp;
    try
    {
      tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
  robot_pos_.y = tf_odom_pose.getOrigin().getY();
  robot_pos_.z = tf_odom_pose.getOrigin().getZ();
  // extract robot heading
  FARUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    // system start time
    FARUtil::systemStartTime = ros::Time::now().toSec();
    FARUtil::map_origin = robot_pos_;
    map_handler_.UpdateRobotPosition(robot_pos_);
  }

  is_odom_init_ = true;
}

void FARMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                             const PointCloudPtr& cloudOut) 
{

  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::fromROSMsg(*pc, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return;
  FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  FARUtil::RemoveNanInfPoints(cloudOut);
  if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: cloud frame does NOT match with world frame!");
    try
    {
      FARUtil::TransformPCLFrame(cloud_frame, 
                                master_params_.world_frame, 
                                tf_listener_,
                                cloudOut);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Tracking cloud TF lookup: %s",ex.what());
      return;
    }
  }
}

void FARMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& scan_pc) {
  if (master_params_.is_static_env || !is_odom_init_) return;
  this->PrcocessCloud(scan_pc, FARUtil::cur_scan_cloud_);
  scan_handler_.UpdateRobotPosition(robot_pos_);
}

void FARMaster::TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (master_params_.is_static_env) return;
  this->PrcocessCloud(pc, local_terrain_ptr_);
  FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_);
}

void FARMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (!is_odom_init_) {
    ROS_WARN_THROTTLE(1.0, "FARMaster: wait for odom to init.");
    return;
  }
  if (!is_stop_update_) {
    this->PrcocessCloud(pc, temp_cloud_ptr_);
    FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range,
                                                               master_params_.terrain_range,
                                                               FARUtil::kTolerZ));
    FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    if (!master_params_.is_static_env) {
      FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true);
    }
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                    FARUtil::surround_obs_cloud_,
                                    FARUtil::cur_new_cloud_);
  } else { // stop env update
    temp_cloud_ptr_->clear();
    FARUtil::cur_new_cloud_->clear();
  }
  // extract surround cloud
  map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
  map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
  // extract dynamic obstacles
  FARUtil::cur_dyobs_cloud_->clear();
  if (!master_params_.is_static_env && !is_stop_update_) {
    this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_, FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
    if (FARUtil::cur_dyobs_cloud_->size() > FARUtil::kDyObsThred) {
      if (FARUtil::IsDebug) ROS_WARN("FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());
      FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_);
      FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
      FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
      FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim);
    }
    // update world dynamic obstacles
    FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime);
  }
  
  // create and update kdtrees
  FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
  FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

  if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;

  /* visualize clouds */
  planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_);
  planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
  planner_viz_.VizPointCloud(surround_obs_debug_,  FARUtil::surround_obs_cloud_);
  // visualize map grid
  PointStack neighbor_centers, occupancy_centers;
  map_handler_.GetNeighborCeilsCenters(neighbor_centers);
  map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
  planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.ceil_length, map_params_.ceil_height);
  // DBBUG visual raycast grids
  if (!master_params_.is_static_env) {
    scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
    planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
  }
}

void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                         const PointCloudPtr& obsCloudIn, 
                                         const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

void FARMaster::WaypointCallBack(const geometry_msgs::PointStamped& route_goal) {
  if (!is_graph_init_) {
    if (FARUtil::IsDebug) ROS_WARN("FARMaster: wait for v-graph to init before sending any goals");
    return;
  }
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_THROTTLE(1.0, "FARMaster: waypoint published is not on world frame!");
    FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); 
  }
  graph_planner_.UpdateGoal(goal_p);
  // visualize original goal
  planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise; 
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kTerrainRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
TimeMeasure FARUtil::Timer;

/* Global Graph */
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;

/* init static contour graph values */
CTNodeStack  ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;

int main(int argc, char** argv){
  ros::init(argc, argv, "far_planner_node");
  FARMaster dp_node;
  dp_node.Init();
  dp_node.Loop();
}