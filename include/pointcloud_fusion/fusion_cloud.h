
#ifndef PROJECT_fusion_pointcloud_H
#define PROJECT_fusion_pointcloud_H

#include <iostream>
#include <string>
#include <chrono>
#include <ros/ros.h>
#include <ros/timer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>


namespace pointcloud_fusion
{

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef message_filters::sync_policies::ApproximateTime<PointCloudT, PointCloudT> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class FusionCloud
{
private:
  int initialize();
  int readParameters();
  void initializeSubscribers();
  void initializePublishers();

  void callbackCloud(const PointCloudT::ConstPtr& cloud1, const PointCloudT::ConstPtr& cloud2);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Publisher pub_pointcloud_;
  ros::Timer timer_;

  message_filters::Subscriber<PointCloudT> sub_cloud_1_;
  message_filters::Subscriber<PointCloudT> sub_cloud_2_;
  boost::shared_ptr<Sync> sync_;

  PointCloudT fusion_pointcloud_;

  std::string target_frame_;
  std::string topic_pub_pointcloud_;
  std::string topic_sub_pointcloud_device_1_;
  std::string topic_sub_pointcloud_device_2_;

  bool debug_;
  bool running_;
  int rate_;

public:
  FusionCloud(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &priv_nh_ = ros::NodeHandle("~"));

  void stop();
  bool start();
};

}
#endif //#ifndef PROJECT_fusion_pointcloud_H

