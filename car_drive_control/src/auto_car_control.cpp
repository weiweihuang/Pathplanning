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

#define SIM_TIME 2000 // X50ms
#define SIM_LOG_L 2500
	
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

float ref_traj[2][SIM_TIME];	
float ori_traj[3][SIM_TIME];
float log_data[13][SIM_TIME]; //ref(x,y,a), actual(x,y,a), ref(v,steeling), actual(v,steeling),gaspedal, breakpedal, direction
float test_data[4][SIM_TIME]; 

float cur_x, cur_y, cur_vel, cur_ori, gas_pedal_state, break_pedal_state;
int direction_state;

static void quit(int sig)
{
  exit(0);
}

/************************Call back function list **************************/

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

   static int count = 0;
   static float ini_pos_x=0, ini_pos_y=0, ini_angle=0;	
   if (count == 0){
	ini_pos_x = msg->pose.pose.position.x;
	ini_pos_y = msg->pose.pose.position.y;
	cur_vel = sqrtf(msg->twist.twist.linear.x*msg->twist.twist.linear.x+msg->twist.twist.linear.y*msg->twist.twist.linear.y);
	cur_ori = 0;
	cur_x = 0;
	cur_y = 0;
	count++;
   }
   else{
	cur_vel = sqrtf(msg->twist.twist.linear.x*msg->twist.twist.linear.x+msg->twist.twist.linear.y*msg->twist.twist.linear.y);
	cur_ori = (tf::getYaw(msg->pose.pose.orientation)+3.14159);
	if (cur_ori > 3.14159)
		cur_ori = cur_ori - 2* 3.14159;
	cur_x = msg->pose.pose.position.x - ini_pos_x;
	cur_y = msg->pose.pose.position.y - ini_pos_y;
   }		

			

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

void gasPedalCallback(const std_msgs::Float64::ConstPtr& msg)
{
	
	gas_pedal_state = msg->data;
}

void breakPedalCallback(const std_msgs::Float64::ConstPtr& msg)
{
	
	break_pedal_state = msg->data;
}

void directionCallback(const std_msgs::Int8::ConstPtr& msg)
{
	
	direction_state = msg->data;
}

void wheelCallback(const std_msgs::Float64::ConstPtr& msg)
{

	std_msgs::Float64 gasPedal, handWheel, breakPedal;
	std_msgs::Int8 direction;
	static int count = 0;
	static int save_flag = 0;
	float cur_wheel = msg->data;
        float vel_error;
	float k_d = 5.0, k_p = 0.1;
	if (carStart){

		if (count==0){

			gasPedal.data = 20*ref_traj[0][count];
			breakPedal.data = 0;
			direction.data = 1;

			log_data[0][count] = ori_traj[0][count];
			log_data[1][count] = ori_traj[1][count];
			log_data[2][count] = ori_traj[2][count];
			log_data[3][count] = cur_x;
			log_data[4][count] = cur_y;
			log_data[5][count] = cur_ori;
			log_data[6][count] = ref_traj[0][count];
			log_data[7][count] = ref_traj[1][count];
			log_data[8][count] = cur_vel;
			log_data[9][count] = cur_wheel;
			log_data[10][count] = gas_pedal_state;
			log_data[11][count] = break_pedal_state;
			log_data[12][count] = direction_state;
			test_data[0][count] = 0;
			test_data[1][count] = 0;
			test_data[2][count] = 0;
			test_data[3][count] = 0;			


			count++;
		}
		else{

			//change current and reference position to car coordinate 
			float ref_car_x = ori_traj[0][count]*cosf(cur_ori) + ori_traj[1][count]*sinf(cur_ori);
			float ref_car_y = -ori_traj[0][count]*sinf(cur_ori) + ori_traj[1][count]*cosf(cur_ori);
			float car_x = cur_x*cosf(cur_ori) + cur_y*sinf(cur_ori);
			float car_y = -cur_x*sinf(cur_ori) + cur_y*cosf(cur_ori);



			//================== motion planning part ====================================//


			float desired_vel = ref_traj[0][count] + 0.25*(ref_car_x - car_x);
			float desired_steering = 5.0*ref_traj[1][count] - 2.0*(cur_ori - ori_traj[2][count]) + 0.5*(ref_car_y - car_y);  


			//================== low level control part ==================================//

			vel_error = desired_vel - cur_vel;

			if (vel_error>0){
				gasPedal.data = 2.5*vel_error-0.5*gas_pedal_state;
				breakPedal.data = 0.0;
			}
			else{
				gasPedal.data = 0;
				breakPedal.data = -5.0*vel_error;
			}

			handWheel.data = desired_steering;


                        //========================= end ==============================================//

			log_data[0][count] = ori_traj[0][count];
			log_data[1][count] = ori_traj[1][count];
			log_data[2][count] = ori_traj[2][count];
			log_data[3][count] = cur_x;
			log_data[4][count] = cur_y;
			log_data[5][count] = cur_ori;
			log_data[6][count] = ref_traj[0][count];
			log_data[7][count] = ref_traj[1][count];
			log_data[8][count] = cur_vel;
			log_data[9][count] = cur_wheel;
			log_data[10][count] = gas_pedal_state;
			log_data[11][count] = break_pedal_state;
			log_data[12][count] = direction_state;
			test_data[0][count] = ref_car_x;
			test_data[1][count] = ref_car_y;
			test_data[2][count] = car_x;
			test_data[3][count] = car_y;

			count++;
			if (count>SIM_TIME-1){
				count = SIM_TIME-1;
				ROS_INFO( "test finished");
			}			
		
		}

		car_gasPedal_pub_.publish(gasPedal); 
		car_handWheel_pub_.publish(handWheel); 
		car_breakPedal_pub_.publish(breakPedal);	
	}
	else if (carStop && count > 100 && save_flag==0){

		FILE *my_stream;

		//save data
		my_stream=fopen("/home/weiweihuang/Matlab/Car_Traj/car_log.txt", "w");
		if (my_stream==NULL){
			printf("Cannot open the file");
			fclose(my_stream);
			exit(1);
		}
		for (int i=0; i<SIM_TIME;i++){
			fprintf(my_stream,"%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", log_data[0][i],log_data[1][i],log_data[2][i],log_data[3][i],log_data[4][i],log_data[5][i],log_data[6][i],log_data[7][i],log_data[8][i],log_data[9][i],log_data[10][i],log_data[11][i],log_data[12][i]);
		}
		fclose(my_stream);
		// save test data
		FILE *my_test;

		//save data
		my_test=fopen("/home/weiweihuang/Matlab/Car_Traj/test_log.txt", "w");
		if (my_test==NULL){
			printf("Cannot open the file");
			fclose(my_test);
			exit(1);
		}
		for (int i=0; i<SIM_TIME;i++){
			fprintf(my_test,"%.5f %.5f %.5f %.5f\n", test_data[0][i],test_data[1][i],test_data[2][i],test_data[3][i]);
		}
		fclose(my_test);
		ROS_INFO( "Log data saved");
		save_flag=1;

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
    ros::Subscriber sub_gas_pedal_ = nh.subscribe<std_msgs::Float64>("/drc_vehicle/gas_pedal/state", 100, &gasPedalCallback);
    ros::Subscriber sub_break_pedal_ = nh.subscribe<std_msgs::Float64>("/drc_vehicle/break_pedal/state", 100, &breakPedalCallback);
    ros::Subscriber sub_direction_ = nh.subscribe<std_msgs::Int8>("/drc_vehicle/direction/state", 100, &directionCallback);	
	


    car_handBrake_pub_ = nh.advertise<std_msgs::Float64>("/drc_vehicle/hand_brake/cmd", 100);	
    car_gasPedal_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/gas_pedal/cmd", 100);
    car_handWheel_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/hand_wheel/cmd", 100);
    car_breakPedal_pub_ = nh.advertise<std_msgs::Float64> ("/drc_vehicle/brake_pedal/cmd", 100);
    car_direction_pub_ = nh.advertise<std_msgs::Int8> ("/drc_vehicle/direction/cmd", 100);

    // initial trajectory
    float Degg = 3.14159/3.0f;
    float RR = 1.8f/sinf(Degg);	
   
   for (int i=0; i< SIM_TIME; i++){
	if(i<500){
		ori_traj[0][i] = 0.036*(i+1);
		ori_traj[1][i] = 0.0;
		ori_traj[2][i] = 0.0;
		//if (i>350)
			//ori_traj[2][i] = (float)(i-350)*6.0/(50*3.14159);		
	}

	else if(i<600){
		ori_traj[0][i] = 18.0+1.80+RR*sinf(1.8*Degg*(i-500)/100.0f-Degg);
		ori_traj[1][i] = 1.5*(RR*cosf(1.8*Degg*(i-500)/100.0f-Degg)-1.0f/sinf(Degg));
		ori_traj[2][i] = 0.0;//-((3.14159-2.0f*Degg)*(i-400)/200.0f+Degg-3.14159/2.0f);	
	}
	else if(i<1000){
		ori_traj[0][i] = 21.6 + 0.036*(i+1-600);
		ori_traj[1][i] = 0.0;
		ori_traj[2][i] = 0.0;	
	}

	else if(i<1500){
		ori_traj[0][i] = 36 - 9.0*sinf(-(float)(i-1000)/500.0*3.14159/2.0);
		ori_traj[1][i] = 9.0*cosf(-(float)(i-1000)/500.0*3.14159/2.0) - 9.0;
		ori_traj[2][i] = -(float)(i-1000)/500.0*3.14159/2.0;
	}
	else{
		ori_traj[0][i] = 45.0;
		ori_traj[1][i] = -9.0 - 0.036*(i-1500);
		ori_traj[2][i] = -3.14159/2; 
	}
   }

   for (int i=0; i< SIM_TIME-1; i++){
	ref_traj[0][i] = 20*sqrtf((ori_traj[0][i+1]-ori_traj[0][i])*(ori_traj[0][i+1]-ori_traj[0][i])+(ori_traj[1][i+1]-ori_traj[1][i])*(ori_traj[1][i+1]-ori_traj[1][i]));
	ref_traj[1][i] = atan2f(1.88*20*(ori_traj[2][i+1]-ori_traj[2][i]), ref_traj[0][i]);
   }
    
    ref_traj[0][SIM_TIME-1] = 0;
    ref_traj[1][SIM_TIME-1] = 0;


    for (int i=0; i< SIM_TIME-1; i++){
	for (int j=0; j<13; j++)
		log_data[j][i] = 0;
	for (int k=0; k<4; k++)
		test_data[k][i] = 0;	
    }	
	
    carStartRefresh_ = false;
 

    boost::thread carControlThread(carControlFunc);
    boost::thread_group group;
    group.add_thread(&carControlThread);

    //run callbacks
    ros::spin();
    group.join_all();
  }



