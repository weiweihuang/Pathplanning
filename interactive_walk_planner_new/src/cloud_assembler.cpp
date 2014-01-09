/*
* cloud_assembler.cpp
*
* Created on: April 16, 2013
* Author: Weiwei Huang
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>


using namespace pcl;

namespace cloud_assembler
{

  typedef PointCloud<PointXYZ> PointCloud2;

  class CloudAssembler
  {

    public:
      CloudAssembler();
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    private:
      ros::NodeHandle node_;

      ros::ServiceServer pause_srv_;

      ros::Publisher output_pub_;
      ros::Subscriber scan_sub_;
      ros::Subscriber sub_odom_;
      ros::Subscriber sub_odom_adj_;

      laser_geometry::LaserProjection projector_;
      tf::TransformListener tf_;
      tf::TransformBroadcaster br_;
      tf::Transform transform;

      message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
      tf::MessageFilter<sensor_msgs::LaserScan> * laser_notifier_;

      PointCloud2 assembled_cloud_;
      int buffer_length_;
      std::vector<sensor_msgs::PointCloud2> cloud_buffer_;
      bool assemblerPaused_;

      void addToBuffer(sensor_msgs::PointCloud2 cloud);
      void assembleCloud();
      bool pauseSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);


  };

  CloudAssembler::CloudAssembler()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param("buffer_length", buffer_length_, 200);

    output_pub_ = node_.advertise<sensor_msgs::PointCloud2> ("/assembled_cloud", 100);

    pause_srv_ = node_.advertiseService("/pause_assembler", &CloudAssembler::pauseSrv, this);

    //***********************************************************************************************//
    laser_sub_.subscribe(node_, "/multisense_sl/laser/scan", 100);        
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, "/l_foot", 100);
    laser_notifier_->registerCallback( boost::bind(&CloudAssembler::scanCallback, this, _1) );
    //***********************************************************************************************//
    PointCloud2 clear;
    assembled_cloud_ = clear;
    assemblerPaused_ = false;
  }


  void CloudAssembler::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

    static int count = 0;
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::LaserScan scan2;
    scan2 = *scan;
    // hack to remove the long distance value
    scan2.range_min = 1.2f;
    scan2.range_max = 10.0f;
    try{

            tf_.waitForTransform("/l_foot", scan->header.frame_id,ros::Time::now(), ros::Duration(1.0));
        projector_.transformLaserScanToPointCloud("/l_foot", scan2, cloud, tf_);

    }

    catch (tf::TransformException& e)
    {
      std::cout << e.what();
      return;
    }

    addToBuffer(cloud);

    if (count ==50){
      count = 0;         
      assembleCloud();
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(assembled_cloud_, cloud_msg);
      cloud_msg.header.frame_id = cloud.header.frame_id;
      cloud_msg.header.stamp = ros::Time::now();
      output_pub_.publish(cloud_msg);
    }
    count++;
  }

  void CloudAssembler::assembleCloud()
  {
    ROS_DEBUG("Assembling.");

    unsigned int i;

    if (assemblerPaused_)
    {
      ROS_INFO("assemblerPaused_ is true");
    }
    if (!assemblerPaused_)
    {
      ROS_DEBUG("assemblerPaused_ is false");
    }

    std::string fixed_frame = cloud_buffer_[0].header.frame_id;

    PointCloud2 new_cloud;
    new_cloud.header.frame_id = fixed_frame;
    new_cloud.header.stamp = ros::Time::now();

    for (i = 0; i < cloud_buffer_.size(); i++)
    {
      PointCloud2 temp_cloud;
      pcl::fromROSMsg(cloud_buffer_[i], temp_cloud);
      temp_cloud.header.frame_id = fixed_frame;
      new_cloud += temp_cloud;
    }

    // If it's paused, don't overwrite the stored cloud with a new one, just keep publishing the same cloud
    if (!assemblerPaused_)
    {
      assembled_cloud_ = new_cloud;
    }
    else if (assemblerPaused_)
    {
      ROS_DEBUG("The Assembler will continue to publish the same cloud.");
    }

  }

  bool CloudAssembler::pauseSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
  {
    ROS_DEBUG("In service call: %s", assemblerPaused_?"true":"false");

    if (!assemblerPaused_)
    {
      ROS_DEBUG("Now paused.");
      assemblerPaused_ = true;
    }
    else if (assemblerPaused_)
    {
      assemblerPaused_ = false;
      ROS_DEBUG("Unpaused.");
    }

    return true;
  }

  void CloudAssembler::addToBuffer(sensor_msgs::PointCloud2 cloud)
  {
    ROS_DEBUG("Adding cloud to buffer. Current buffer length is %d", cloud_buffer_.size());

    if (cloud_buffer_.size() >= (unsigned int)buffer_length_)
    {
      cloud_buffer_.erase(cloud_buffer_.begin());
    }

    cloud_buffer_.push_back(cloud);
  }




}; // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cloud_assembler");
  cloud_assembler::CloudAssembler cloud_assembler;

  ros::spin();

  return 0;
}
