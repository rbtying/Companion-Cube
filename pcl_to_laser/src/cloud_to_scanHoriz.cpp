#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


namespace pcl_to_laser
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudToScanHoriz : public nodelet::Nodelet
{
public:
  //Constructor
  CloudToScanHoriz(): min_height_(0.10), max_height_(0.75), baseFrame("/base_footprint"), laserFrame("/camera_tower")
  {
  };

private:
  double min_height_, max_height_;
  std::string baseFrame;  //ground plane referenced by laser
  std::string laserFrame; //pan frame simulating projected laser
  bool result;

  tf::TransformListener tfListener;
  message_filters::Subscriber<PointCloud> cloudSubscriber;
  tf::MessageFilter<PointCloud> *tfFilter;

  ros::Publisher laserPublisher;

  //Nodelet initialization
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);
    private_nh.getParam("base_frame", baseFrame);
    private_nh.getParam("laser_frame", laserFrame);
    NODELET_INFO("CloudToScanHoriz min_height: %f, max_height: %f", min_height_, max_height_);
    NODELET_INFO("CloudToScanHoriz baseFrame: %s, laserFrame: %s", baseFrame.c_str(), laserFrame.c_str());

    //Set up to process new pointCloud and tf data together
    cloudSubscriber.subscribe(nh, "cloud_in", 5);
    tfFilter = new tf::MessageFilter<PointCloud>(cloudSubscriber, tfListener, laserFrame, 1);
    tfFilter->registerCallback(boost::bind(&CloudToScanHoriz::callback, this, _1));
    tfFilter->setTolerance(ros::Duration(0.01));

    laserPublisher = nh.advertise<sensor_msgs::LaserScan>("laserScanHoriz", 10);
  };

  //Pointcloud and tf transform received
  void callback(const PointCloud::ConstPtr& cloud_in)
  {
    PointCloud cloudTransformed;
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());

    try
    {
      //Transform pointcloud to new reference frame
      result = pcl_ros::transformPointCloud(baseFrame, *cloud_in, cloudTransformed, tfListener);
    }
    catch (tf::TransformException& e)
    {
      NODELET_INFO("CloudToScanHoriz failed");
      std::cout << e.what();
      return;
    }
    //NODELET_DEBUG("Got cloud");

    //Setup laserscan message
    output->header = cloud_in->header;
    output->header.frame_id = laserFrame;
    output->angle_min = -M_PI/2;
    output->angle_max = M_PI/2;
    output->angle_increment = M_PI/180.0/2.0;
    output->time_increment = 0.0;
    output->scan_time = 1.0/30.0;
    output->range_min = 0.45;
    output->range_max = 10.0;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    //"Thin" laser height from pointcloud
    for (PointCloud::const_iterator it = cloudTransformed.begin(); it != cloudTransformed.end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        continue;  
      }
      if (z > max_height_ || z < min_height_)
      {
        continue;
      }
      double angle = atan2(y, x);
      if (angle < output->angle_min || angle > output->angle_max)
      {
        continue;
      }
      int index = (angle - output->angle_min) / output->angle_increment;
      //Calculate hypoteneuse distance to point
      double range_sq = y*y+x*x;
      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
    } //for it
    laserPublisher.publish(output);
  } //callback

};

PLUGINLIB_DECLARE_CLASS(pcl_to_laser, CloudToScanHoriz, pcl_to_laser::CloudToScanHoriz, nodelet::Nodelet);
}
