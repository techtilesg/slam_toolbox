#include "slam_toolbox/map_editor_server.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
MapEditor::MapEditor(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  std::string filename;
  geometry_msgs::Pose2D pose;
  bool dock = true;
  map_editor_mode_ = true;
  if (shouldStartWithPoseGraph(filename, pose, dock))
  {
    slam_toolbox_msgs::DeserializePoseGraph::Request req;
    slam_toolbox_msgs::DeserializePoseGraph::Response resp;
    req.initial_pose = pose;
    req.filename = filename;
    req.match_type =
      slam_toolbox_msgs::DeserializePoseGraph::Request::START_AT_FIRST_NODE;
    deserializePoseGraphCallback(req, resp);
  }


  // interactive mode
  processor_type_ = PROCESS;
  enable_interactive_mode_ = true;

  // in localization mode, disable map saver
  map_saver_.reset();
  return;
}

/*****************************************************************************/
bool MapEditor::serializePoseGraphCallback(
  slam_toolbox_msgs::SerializePoseGraph::Request& req,
  slam_toolbox_msgs::SerializePoseGraph::Response& resp)
/*****************************************************************************/
{
  ROS_FATAL("MapEditor: Cannot call serialize map "
    "in localization mode!");
  return false;
}

/*****************************************************************************/
bool MapEditor::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
//  if (req.match_type != procType::LOCALIZE_AT_POSE)
//  {
//    ROS_ERROR("Requested a non-localization deserialization "
//      "in localization mode.");
//    return false;
//  }
  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

/*****************************************************************************/
void MapEditor::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  {
    boost::mutex::scoped_lock lock(last_scan_mutex_);
    last_scan_stamp_ = ros::Time::now();
    last_scan_header_ = scan->header;
  }
  // no odom info
  Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (shouldProcessScan(scan, pose))
  {
    addScan(laser, scan, pose);
  }
  
  return;
}

/*****************************************************************************/
LocalizedRangeScan* MapEditor::addScan(
  LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan,
  Pose2& karto_pose)
/*****************************************************************************/
{
  ros::WallTime start = ros::WallTime::now();
  boost::mutex::scoped_lock l(pose_mutex_);

  if (processor_type_ == PROCESS_LOCALIZATION && process_near_pose_)
  {
    processor_type_ = PROCESS_NEAR_REGION;
  }

  LocalizedRangeScan* range_scan = getLocalizedRangeScan(
    laser, scan, karto_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  if (processor_type_ == PROCESS_NEAR_REGION)
  {
    if (!process_near_pose_)
    {
      ROS_ERROR("Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }

    // set our position to the requested pose and process
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan, true, &covariance);

    // reset to localization mode
    processor_type_ = PROCESS_LOCALIZATION;
    update_reprocessing_transform = true;

    double dur = (ros::WallTime::now() - start).toSec() * 1e3;
    ROS_DEBUG("ProcessAgainstNodesNearBy: %.1f ms", dur);
  }
  else if (processor_type_ == PROCESS_LOCALIZATION)
  {
    processed = smapper_->getMapper()->ProcessLocalization(range_scan,  &covariance);
    update_reprocessing_transform = false;
    double dur = (ros::WallTime::now() - start).toSec() * 1e3;
    ROS_DEBUG("addScan: ProcessLocalization: %.1f ms", dur);
  }
  else
  {
    ROS_FATAL("MapEditor: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  if(!processed)
  {
    delete range_scan;
    range_scan = nullptr;
  } else {
    // compute our new transform
    setTransformFromPoses(range_scan->GetCorrectedPose(), karto_pose,
      scan->header.stamp, update_reprocessing_transform);

    publishPose(range_scan->GetCorrectedPose(), covariance, scan->header.stamp);
  }
  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("addScan: end %.1f ms", dur);

  return range_scan;
}

/*****************************************************************************/
void MapEditor::localizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  boost::mutex::scoped_lock l(pose_mutex_);
  if (process_near_pose_)
  {
    process_near_pose_.reset(new Pose2(msg->pose.pose.position.x, 
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)));
  }
  else
  {
    process_near_pose_ = std::make_unique<Pose2>(msg->pose.pose.position.x, 
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));    
  }

  first_measurement_ = true;

  boost::mutex::scoped_lock lock(smapper_mutex_);
  smapper_->clearLocalizationBuffer();

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
  return;
}

} // end namespace
