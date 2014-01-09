/*
 * auto_car_control.cpp
 *
 *  Created on: Dec 20, 2013
 *      Author: Weiwei Huang
 */


#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread/thread.hpp>
#include "tf/transform_datatypes.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <Eigen/Core>

#define SIM_TIME 1200 // ms
	
volatile bool carStartRefresh_;

ros::Time initTime;


static ros::Publisher car_handBrake_pub_;
static ros::Publisher car_gasPedal_pub_;
static ros::Publisher car_handWheel_pub_;
static ros::Publisher car_breakPedal_pub_;
static ros::Publisher car_direction_pub_;
static ros::Publisher odom_pub_;


bool carStart = false;
bool carStop = true;

float gas_p = 0.1;
float K_p = 0.1;
float K_d = 5.0;

float ref_traj[3][SIM_TIME];	
float ori_traj[2][SIM_TIME];

float cur_vel;

static void quit(int sig)
{
  exit(0);
}

/************************Call back function list **************************/

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

   cur_vel = sqrtf(msg->twist.twist.linear.x*msg->twist.twist.linear.x+msg->twist.twist.linear.y*msg->twist.twist.linear.y);	

/*
   std_msgs::Float64 gasPedal, handWheel, breakPedal;
   std_msgs::Int8 direction;
   static float ini_pos_x=0, ini_pos_y=0, ini_angle=0;
   float cur_pos_x, cur_pos_y, cur_angle;
   float cur_vel_x, cur_vel_y, cur_vel_angle;

   static int count = 0;
   static int print_count = 0;	

   if (carStart){

	if (count==0){
		ini_pos_x = msg->pose.pose.position.x;
		ini_pos_y = msg->pose.pose.position.y;
		ini_angle = tf::getYaw(msg->pose.pose.orientation);

		gasPedal.data = 1000*ori_traj[0][count];
		breakPedal.data = 0;
		direction.data = 1;
		count++;
	}
	else{
		handWheel.data = 0.1f;
		cur_pos_x = msg->pose.pose.position.x;
		cur_pos_y = msg->pose.pose.position.y;
		cur_angle = tf::getYaw(msg->pose.pose.orientation);

		cur_vel_x = msg->twist.twist.linear.x;
		cur_vel_y = msg->twist.twist.linear.y;
		cur_vel_angle = msg->twist.twist.angular.z;


		float gas_from_x = gas_p*1000*(ori_traj[0][count]-ori_traj[0][count-1]) + K_p*1000*(ori_traj[0][count-1] - (cur_pos_x-ini_pos_x)) - K_d*cur_vel_x;
		float gas_from_y = gas_p*1000*(ori_traj[1][count]-ori_traj[1][count-1]) + K_p*1000*(ori_traj[1][count-1] - (cur_pos_y-ini_pos_y)) - K_d*cur_vel_y; 

		gasPedal.data = gas_from_x + gas_from_y;

		float desired_phi = atan2f(sqrtf(cur_vel_x*cur_vel_x+cur_vel_y*cur_vel_x),1.88*1000*(ori_traj[2][count]-ori_traj[2][count-1]));

		handWheel.data = desired_phi + K_p*(ori_traj[2][count]-cur_angle) - K_d*cur_vel_angle;
          
                if (gasPedal.data < 0){
			gasPedal.data = -gasPedal.data;
			direction.data = -1;
		}
		else
			direction.data = 1;	
		count++;
		if (count>SIM_TIME-1)
			count = SIM_TIME-1;
	    			
	}
        car_direction_pub_.publish(direction); 
	car_gasPedal_pub_.publish(gasPedal); 
	car_handWheel_pub_.publish(handWheel); 

	print_count++;		     
   }		
*/
}

void wheelCallback(const std_msgs::Float64::ConstPtr& msg)
{

	std_msgs::Float64 gasPedal, handWheel, breakPedal;
	std_msgs::Int8 direction;
	static int count = 0;
	float cur_wheel = msg->data;
        float vel_error;
	float k_d = 5.0, k_p = 0.1;
	if (carStart){
		if (count==0){

			gasPedal.data = 50*ori_traj[0][count];
			breakPedal.data = 0;
			direction.data = 1;
			count++;
		}
		else{
			vel_error = ori_traj[0][count] - cur_vel;
			if (vel_error>0){
				gasPedal.data = 0.5*vel_error;
				breakPedal.data = 0;	
			}
			else{
				breakPedal.data = 0.3*vel_error;
				gasPedal.data = 0;	
			}

			handWheel.data = (ori_traj[1][count] - cur_wheel);
			count++;
			if (count>SIM_TIME-1)
				count = SIM_TIME-1;
		
		}
		ROS_INFO( "desired vel is %g, and actual vel is %g", ori_traj[0][count], cur_vel);
		ROS_INFO( "desired wheel is %g, and actual wheel is %g", ori_traj[1][count], cur_wheel);
		car_gasPedal_pub_.publish(gasPedal); 
		car_handWheel_pub_.publish(handWheel); 
		car_breakPedal_pub_.publish(breakPedal);	
	}

}

/************************ main function list **************************/

void carControlFunc()
{

  usleep(1000000);
  std_msgs::Float64 handBrake, gasPedal, handWheel, breakPedal;
  char higher_level, speed_control;


  

  handBrake.data = 1.0f;
  gasPedal.data = 0.0f;	
  handWheel.data = 0.0f;
  breakPedal.data = 0.0f;

  while(1){

	  if(carStop){
		  ROS_INFO( "Shall we start the car");
		  scanf("  %c", &higher_level );
		  if (higher_level=='y'){
			carStart = true;
			carStop = false;
			handBrake.data = 0.0f;
			gasPedal.data = 0.0f;
			breakPedal.data = 0.0f;
			  car_handBrake_pub_.publish(handBrake);
			  car_gasPedal_pub_.publish(gasPedal);
			  car_handWheel_pub_.publish(handWheel);
			  car_breakPedal_pub_.publish(breakPedal);
			  usleep(1000);	
		  }
			
	  }
	  if (carStart){

		  scanf("  %c", &speed_control );		  
		  if (speed_control=='n'){
			handBrake.data = 1.0;
			gasPedal.data = 0.0;
			carStart = false;
			carStop = true;
			  car_handBrake_pub_.publish(handBrake);
			  car_gasPedal_pub_.publish(gasPedal);
			  car_handWheel_pub_.publish(handWheel);
			  car_breakPedal_pub_.publish(breakPedal);
			  usleep(1000);
		  }
               
	}
  }	

}

int main(int argc, char **argv)
  {
    signal(SIGINT,quit);
    ros::init(argc, argv, "auto_car_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::cout << "system starts" << std::endl;

    initTime = ros::Time::now();


    ros::Subscriber sub_odom_ = nh.subscribe<nav_msgs::Odometry>("/car_truth_odom", 100, &odomCallback);
    ros::Subscriber sub_wheel_ = nh.subscribe<std_msgs::Float64>("/drc_vehicle/hand_wheel/state", 100, &wheelCallback);


    car_handBrake_pub_ = nh.advertise<std_msgs::Float64>("/drc_vehicle/hand_brake/cmd", 100);	
    car_gasPedal_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/gas_pedal/cmd", 100);
    car_handWheel_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/hand_wheel/cmd", 100);
    car_breakPedal_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/brake_pedal/cmd", 100);
    car_direction_pub_ = nh.advertise<std_msgs::Int8> ("/drc_vehicle/direction/cmd", 100);

    // initial trajectory
   
   for (int i=0; i< SIM_TIME; i++){
	ori_traj[0][i] = 0.01*(i+1);
	ori_traj[1][i] = 0.0;
	ori_traj[2][i] = 0.0;	
   }

   for (int i=0; i< SIM_TIME-1; i++){
	ref_traj[0][i] = 50*sqrtf((ori_traj[0][i+1]-ori_traj[0][i])*(ori_traj[0][i+1]-ori_traj[0][i])+(ori_traj[1][i+1]-ori_traj[1][i])*(ori_traj[1][i+1]-ori_traj[1][i]));
	ref_traj[1][i] = atan2f(ref_traj[0][i], 1.88*50*(ori_traj[2][i+1]-ori_traj[2][i]));
   }
    
    ref_traj[0][SIM_TIME-1] = 0;
    ref_traj[1][SIM_TIME-1] = 0;
	
    carStartRefresh_ = false;
 

    boost::thread carControlThread(carControlFunc);
    boost::thread_group group;
    group.add_thread(&carControlThread);

    //run callbacks
    ros::spin();
    group.join_all();
  }



