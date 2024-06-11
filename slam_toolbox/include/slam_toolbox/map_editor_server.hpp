#ifndef SLAM_TOOLBOX_MAP_EDITOR_H_
#define SLAM_TOOLBOX_MAP_EDITOR_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

using namespace ::karto;

class MapEditor : public SlamToolbox
{
public:
  MapEditor(ros::NodeHandle& nh);
  ~MapEditor() {};

protected:
  virtual void laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  void localizePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  virtual bool serializePoseGraphCallback(
    slam_toolbox_msgs::SerializePoseGraph::Request& req,
    slam_toolbox_msgs::SerializePoseGraph::Response& resp) override final;
  virtual bool deserializePoseGraphCallback(
    slam_toolbox_msgs::DeserializePoseGraph::Request& req,
    slam_toolbox_msgs::DeserializePoseGraph::Response& resp) override final;

  virtual LocalizedRangeScan* addScan(karto::LaserRangeFinder* laser,
    const sensor_msgs::LaserScan::ConstPtr& scan,
    karto::Pose2& karto_pose) override final;

  ros::Subscriber localization_pose_sub_;
};

}

#endif //SLAM_TOOLBOX_MAP_EDITOR_H_
