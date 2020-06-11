//
// Created by riestock on 09.04.19.
//
#ifndef fusion_pointcloud
#define fusion_pointcloud

#include <pointcloud_fusion/fusion_cloud.h>
#include <nodelet/nodelet.h>


namespace pointcloud_fusion
{

/// pointcloud_fusion takes two topics from pointsclouds and adds all points from them into one cloud
/// this fusioned pointcloud is then published into a new topic
/// \param nh_ Nodehandle used for publisher and subscriber
/// \param priv_nh_ Nodehandle used for parameter handling from roslaunch
FusionCloud::FusionCloud(const ros::NodeHandle &nh_, const ros::NodeHandle &priv_nh_) :
    nh_(nh_),
    priv_nh_(priv_nh_),
    target_frame_(""),
    topic_pub_pointcloud_(""),
    topic_sub_pointcloud_device_1_(""),
    topic_sub_pointcloud_device_2_(""),
    debug_(false),
    running_(false),
    rate_(1)
{}

bool FusionCloud::start()
{
  if(running_)
  {
    ROS_ERROR("[FusionCloud] is already running!");
    return false;
  }
  if(!initialize())
  {
    ROS_ERROR("[FusionCloud] Initialization failed!");
    return false;
  }
  running_ = true;

  return true;
}

void FusionCloud::stop()
{
  if(!running_)
  {
    ROS_ERROR("[FusionCloud] is not running!");
    return;
  }
  running_ = false;

  pub_pointcloud_.shutdown();
  nh_.shutdown();
  priv_nh_.shutdown();
}

/// calls all sub-initalize functions
/// \return true if all sub-initalize functions were successful
int FusionCloud::initialize()
{
  if(!readParameters())
  {
    ROS_ERROR("[FusionCloud] Could not read parameters!");
  }

  initializeSubscribers();
  initializePublishers();
  return true;
}

/// reads the parameter from roslaunch
/// \return true if all parameters were successfully received
int FusionCloud::readParameters()
{
  return
      priv_nh_.getParam("debug", debug_) &&
      priv_nh_.getParam("rate", rate_) &&
      priv_nh_.getParam("target_frame", target_frame_) &&
      priv_nh_.getParam("topic_sub_pointcloud_device_1", topic_sub_pointcloud_device_1_) &&
      priv_nh_.getParam("topic_sub_pointcloud_device_2", topic_sub_pointcloud_device_2_) &&
      priv_nh_.getParam("topic_pub_pointcloud", topic_pub_pointcloud_);
}


void FusionCloud::initializeSubscribers()
{
  ROS_INFO("[FusionCloud] Initializing Subscribers");
  sub_cloud_1_.subscribe(nh_, topic_sub_pointcloud_device_1_, 10);
  sub_cloud_2_.subscribe(nh_, topic_sub_pointcloud_device_2_, 10);

  sync_.reset(new Sync(MySyncPolicy(10), sub_cloud_1_, sub_cloud_2_));
  sync_->registerCallback(boost::bind(&pointcloud_fusion::FusionCloud::callbackCloud, this, _1, _2));
}

void FusionCloud::initializePublishers()
{
  ROS_INFO("[FusionCloud] Initializing Publishers");
  pub_pointcloud_ = nh_.advertise<PointCloudT>(topic_pub_pointcloud_, 10);
}

/// one callback for both pointclouds and topics
/// this callback receives both pointclouds, generates a new pointcloud from the points of the
/// received pointclouds and also publishes inside this function
void FusionCloud::callbackCloud(const PointCloudT::ConstPtr& cloud1, const PointCloudT::ConstPtr& cloud2)
{
  auto start = std::chrono::high_resolution_clock::now();

  int diff = cloud1->header.stamp - cloud2->header.stamp;
  ROS_INFO_COND(debug_, "[FusionCloud] Diff %i microseconds", diff);

  if(cloud1->header.frame_id == cloud2->header.frame_id)
  {
    fusion_pointcloud_.clear();
    fusion_pointcloud_.header.frame_id = cloud1->header.frame_id;
    fusion_pointcloud_.header.stamp = cloud1->header.stamp;
    fusion_pointcloud_ += *cloud1;
    fusion_pointcloud_ += *cloud2;
    pub_pointcloud_.publish(fusion_pointcloud_);
  }
  else
  {
    ROS_ERROR("[FusionCloud] Received different header -> No pointcloud_fusion!");
    ROS_ERROR("%s || %s", cloud1->header.frame_id.c_str(), cloud2->header.frame_id.c_str());
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  ROS_INFO_COND(debug_, "[FusionCloud] Time taken by function: %li microseconds", duration.count());
}

/// nodelet class which inizalises the base class. Is necessary for intraprocess communication.
class FusionCloudNodelet : public nodelet::Nodelet
{
private:
  FusionCloud *pFusionCloud;

public:
  FusionCloudNodelet() : Nodelet(), pFusionCloud(nullptr)
  {
    NODELET_INFO("[FusionCloudNodelet] Constructor call");
  }

  ~FusionCloudNodelet() override
  {
    NODELET_INFO("[FusionCloudNodelet] Destructor call");
    if(pFusionCloud)
    {
      NODELET_INFO("[FusionCloudNodelet] Have something to destruct");
      pFusionCloud->stop();
      delete pFusionCloud;
    }
  }

  void onInit() override
  {
    NODELET_INFO("[FusionCloudNodelet] onInit");
    pFusionCloud = new FusionCloud(getNodeHandle(), getPrivateNodeHandle());
    if(!pFusionCloud->start())
    {
      delete pFusionCloud;
      pFusionCloud = nullptr;
      throw nodelet::Exception("[FusionCloudNodelet] Could not start nodelet");
    }
  }
};

} // namespace pointcloud_fusion


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pointcloud_fusion::FusionCloudNodelet, nodelet::Nodelet)

/// allows to include this file with main function into a library. hack, can breake.
#ifndef fusion_EXPORTS

/// init the base class if not used as nodelet
int main(int argc, char **argv)
{

  ros::init(argc, argv, "pointcloud_fusion", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    ROS_ERROR("ros::ok failed!");
    return -1;
  }

  pointcloud_fusion::FusionCloud FusionCloud;
  if(FusionCloud.start())
  {
    ros::spin();

    FusionCloud.stop();
  }

  ros::shutdown();
  return 0;
}

#endif // fusion_pointcloud_EXPORTS

#endif // fusion_pointcloud