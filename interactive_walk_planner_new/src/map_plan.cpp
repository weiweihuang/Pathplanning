/*
 * ===========================================================================
 *
 *       Filename:  map_plan_online.cpp
 *
 *    Description:  build the height map for path planning
 *
 *        Version:  1.0
 *        Created:  22/04/2013 09:47:29 AM
 *       Revision:  none
 *       Compiler:  rosmake
 *
 *         Author:  Weiwei Huang, 
 *        Company:  CMU
 *
 * ===========================================================================
 */

/*************************************************************************/

#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>

#include <atlas_ros_msgs/field_command.h>
#include <atlas_ros_msgs/field_state.h>

// for offline stepping
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>

#include <atlas_ros_msgs/AtlasStepFeedback.h>
#include <atlas_ros_msgs/AtlasStepParams.h>
#include <atlas_ros_msgs/SimpleWalkParams.h>

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
#include <interactive_walk_planner/foot_sequence.h>
#include <boost/thread/thread.hpp>
#include "tf/transform_datatypes.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <Eigen/Core>

#include "astar.h"
//#include "dirDetector.h"
#include <interactive_walk_planner/goal_pose.h>
#include <interactive_walk_planner/replan.h>

#define MAX_FOOT_NO 50
/*************************************************************************/

volatile bool NewclouddataRefresh_;
volatile bool planNeedsRefresh_;
volatile bool AlignNeedsRefresh_;
volatile bool NoStopMapping_;
volatile bool GoalposeUpdated_;
volatile bool viewerNeedsRefresh_;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sc_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
static pthread_mutex_t mtx_, refreshMtx_, terrain_mutex;
CvMat *local_terrain=cvCreateMat(TERRAIN_N_X,TERRAIN_N_Y,CV_8UC3);
CvMat *show_terrain=cvCreateMat(TERRAIN_N_X,TERRAIN_N_Y,CV_8UC3);
float terrain_value[TERRAIN_N_X][TERRAIN_N_Y];
float normal_value[TERRAIN_N_X][TERRAIN_N_Y][3];
float curr_normal_value[TERRAIN_N_X][TERRAIN_N_Y][3];

float goal_pos_x = 0, goal_pos_y = 0;
int plan_method = 0;

int task_mode=0;
std::string ActionPath;


//#define DRAW
//#define NORMAL

using namespace cv;
const char* image_window = "Source Image";

ros::Time initTime;


static ros::Publisher output_pub_;
static ros::Publisher foot_sequence_pub_;

static void quit(int sig)
{
  exit(0);
}

/************************seperate function list******************************/

int getPose2DFast(Mat origin_image, Point2f & center, double * pfDir)
{

  double  minDir, tmpDir;

  Point minLoc1, tmpLoc1, maxLoc1;

  double lastVal, minVal = 1.0e7, tmpVal, maxVal;
  
  Mat pattern_image = Mat(30, 30, CV_8UC1);

  Mat standard_pattern = Mat(30, 30, CV_8UC1, Scalar(255));

  Mat small_pattern = Mat(20, 20, CV_8UC1, Scalar(0));

  Mat result1;

  if(origin_image.empty())
  {
     printf("fail to open image to read\n");
     return -1;
  }

  small_pattern = round(origin_image.at<unsigned char>(round(center.y), round(center.x))/5.0 +  \
                   origin_image.at<unsigned char>(round(center.y-1), round(center.x))/5.0 +     \
                   origin_image.at<unsigned char>(round(center.y+1), round(center.x))/5.0 +      \
                   origin_image.at<unsigned char>(round(center.y), round(center.x - 1))/5.0 +    \
                   origin_image.at<unsigned char>(round(center.y-1), round(center.x + 1))/5.0    \     
                   );

  small_pattern.copyTo(standard_pattern(Rect(5,5,20,20)));

   

  

  Rect subRegion;
  subRegion.x = round(center.x - 20);

  if( subRegion.x < 0) 
  {
      subRegion.x = 0;
  }
  subRegion.y = round(center.y - 20);
  if(subRegion.y < 0)
  {
     subRegion.y = 0;
  }
  subRegion.width = origin_image.cols - subRegion.x ;

  if( subRegion.width > 40) 
  {
    subRegion.width = 40;
  }
  
  subRegion.height =  origin_image.rows - subRegion.y;
  if(subRegion.height > 40)
  {
    subRegion.height = 40;
  }
    
  
  int a = round(sqrt( subRegion.height * subRegion.height + subRegion.width * subRegion.width)  + 5);
  Mat proc_image = Mat(subRegion.height, subRegion.width, CV_8UC1);

  origin_image(subRegion).copyTo(proc_image);

  

  Mat ext_proc_image = Mat(a, a, CV_8UC1);

  ext_proc_image = 255;

  
  proc_image.copyTo(ext_proc_image(  Rect( round((a - subRegion.width)/2.0), round(( a - subRegion.height)/2.0), subRegion.width, subRegion.height)))  ;  

  
  Mat RotImage = Mat(a, a, CV_8UC1, Scalar(255));

  int result1_cols =  RotImage.cols - small_pattern.cols + 1;
   int result1_rows = RotImage.rows - small_pattern.rows + 1; 

   result1.create( result1_rows, result1_cols, CV_64FC1 );

  for(int i = 0; i < 180; i++)
  {

     RotImage = 255;
    
     Mat affine_mat = getRotationMatrix2D(Point2f(a/2.0, a/2.0), i, 1);

       warpAffine(ext_proc_image, RotImage, affine_mat, RotImage.size(),CV_INTER_LINEAR, BORDER_TRANSPARENT );

    
     
     
    result1 = .0;

     matchTemplate( RotImage, small_pattern, result1, TM_SQDIFF );

     minMaxLoc( result1, &tmpVal, &maxVal, &tmpLoc1, &maxLoc1, Mat() );

     if ( tmpVal < minVal)
     {
        minVal = tmpVal;
        minLoc1 = tmpLoc1;
        minDir = i;
     }    


  }

  
  double theta = minDir / 180.0 * CV_PI;
  center.x = cos( theta) *  (minLoc1.x + 9.5 - a/2.0) - sin(theta) * (minLoc1.y + 9.5 - a/2.0) + subRegion.x + subRegion.width/2.0;
  center.y = sin( theta) *  (minLoc1.x + 9.5 - a/2.0) + cos(theta) * (minLoc1.y + 9.5 - a/2.0) + subRegion.y + subRegion.height/2.0;

  *pfDir = minDir;


  // just for velicication usage. If not needed, change 1 to 0
  #if 1

  {

     pattern_image = 255;
    
     Mat affine_mat = getRotationMatrix2D(Point2f(14.5, 14.5), -minDir, 1);

       warpAffine(standard_pattern, pattern_image, affine_mat, pattern_image.size() ,CV_INTER_LINEAR, BORDER_TRANSPARENT );

    
      Mat result_image = origin_image.clone();

     
     min(pattern_image, result_image(Rect(round(center.x - 14.5), round(center.y - 14.5), 30, 30)), pattern_image);

     pattern_image.copyTo(result_image(Rect(round(center.x - 14.5), round(center.y - 14.5), 30, 30)));

    imwrite("/home/huangwei/resultImg.jpg", result_image);

     


  }
  #endif

  return 0;



}

int getPose2D(Mat origin_image, Point2f & center, double * pfDir)
{

  double  minDir, tmpDir;

  Point minLoc1, tmpLoc1, maxLoc1;

  double lastVal, minVal = 1.0e7, tmpVal, maxVal;
  
  Mat pattern_image = Mat(30, 30, CV_8UC1);

  Mat standard_pattern = Mat(30, 30, CV_8UC1, Scalar(255));

  Mat small_pattern = Mat(20, 20, CV_8UC1, Scalar(0));
  

  Mat result1;

  small_pattern.copyTo(standard_pattern(Rect(5,5,20,20)));

  

  Rect subRegion;
  subRegion.x = round(center.x - 40);

  if( subRegion.x < 0) 
  {
      subRegion.x = 0;
  }
  subRegion.y = round(center.y - 40);
  if(subRegion.y < 0)
  {
     subRegion.y = 0;
  }
  subRegion.width = origin_image.cols - subRegion.x ;

  if( subRegion.width > 80) 
  {
    subRegion.width = 80;
  }
  
  subRegion.height =  origin_image.rows - subRegion.y;
  if(subRegion.height > 80)
  {
    subRegion.height = 80;
  }
    
  
  int a = round(sqrt( subRegion.height * subRegion.height + subRegion.width * subRegion.width)  + 5);
  Mat proc_image = Mat(subRegion.height, subRegion.width, CV_8UC1);

  origin_image(subRegion).copyTo(proc_image);

 
  Mat ext_proc_image = Mat(a, a, CV_8UC1);

  ext_proc_image = 255;

  proc_image.copyTo(ext_proc_image(  Rect( round((a - subRegion.width)/2.0), round(( a - subRegion.height)/2.0), subRegion.width, subRegion.height)))  ;  

  
  Mat RotImage = Mat(a, a, CV_8UC1, Scalar(255));

  int result1_cols =  RotImage.cols - small_pattern.cols + 1;
   int result1_rows = RotImage.rows - small_pattern.rows + 1; 

   result1.create( result1_rows, result1_cols, CV_64FC1 );

  for(int i = 0; i < 180; i++)
  {

     RotImage = 255;
    
     Mat affine_mat = getRotationMatrix2D(Point2f(a/2.0, a/2.0), i, 1);

       warpAffine(ext_proc_image, RotImage, affine_mat, RotImage.size(),CV_INTER_LINEAR, BORDER_TRANSPARENT );
     
     //imwrite("resultImg.jpg", RotImage);

    result1 = .0;

     matchTemplate( RotImage, small_pattern, result1, TM_SQDIFF );

     minMaxLoc( result1, &tmpVal, &maxVal, &tmpLoc1, &maxLoc1, Mat() );

     if ( tmpVal < minVal)
     {
        minVal = tmpVal;
        minLoc1 = tmpLoc1;
        minDir = i;
     }    


  }

  
  double theta = -minDir / 180.0;
  center.x = cos( theta) *  (minLoc1.x + 9.5 - a/2.0) - sin(theta) * (minLoc1.y + 9.5 - a/2.0) + subRegion.x + subRegion.width/2.0;
  center.y = sin( theta) *  (minLoc1.x + 9.5 - a/2.0) + cos(theta) * (minLoc1.y + 9.5 - a/2.0) + subRegion.y + subRegion.height/2.0;

  *pfDir = minDir;


  #if 1

  {
     pattern_image = 255;
    
     Mat affine_mat = getRotationMatrix2D(Point2f(14.5, 14.5), -minDir, 1);

       warpAffine(standard_pattern, pattern_image, affine_mat, pattern_image.size() ,CV_INTER_LINEAR, BORDER_TRANSPARENT );

    
      Mat result_image = origin_image.clone();

     
     min(pattern_image, result_image(Rect(round(center.x - 14.5), round(center.y - 14.5), 30, 30)), pattern_image);

     pattern_image.copyTo(result_image(Rect(round(center.x - 14.5), round(center.y - 14.5), 30, 30)));

    imwrite("/home/huangwei/resultImg.jpg", result_image);

  }
  #endif
   
  return 0;
   

}



void DrawFoot(CvArr* img, CvPoint position, double angle)
{
  CvPoint* pts = new CvPoint[4];
  double pi = 3.1415926;
  double regularAngel = 70*pi/180;
  double r = 4;
  pts[0].x = cos(angle+regularAngel)*r+position.x;
  pts[0].y = sin(angle+regularAngel)*r+position.y;
  pts[1].x = cos(pi-2*regularAngel+angle+regularAngel)*r+position.x;
  pts[1].y = sin(pi-2*regularAngel+angle+regularAngel)*r+position.y;
  pts[2].x = cos(pi+angle+regularAngel)*r+position.x;
  pts[2].y = sin(pi+angle+regularAngel)*r+position.y;
  pts[3].x = cos(angle-regularAngel)*r+position.x;
  pts[3].y = sin(angle-regularAngel)*r+position.y;

  cvFillConvexPoly(img,pts,4,cvScalar(255,0,0,0));
}


void DrawFootSequence (ASTAR *aa, PATH_NODE *pn)
  {
    int index, count = 0;
    interactive_walk_planner::foot_sequence foot_sequence;
    //============== temp test area ======================// 
    volatile bool right_foot;
    right_foot = false;

    while (pn != NULL)
    {
      if ( pn->state[XX] < aa->tt->min[XX]
          || pn->state[XX] > aa->tt->max[XX]
          || pn->state[YY] < aa->tt->min[YY]
          || pn->state[YY] > aa->tt->max[YY] ){
        pn = pn->previous;
        continue;
      }
      else{
#ifdef DRAW
        CvPoint position;
        position.x = (int)((pn->state[XX]-aa->tt->min[XX])/DISTANCE);
        position.y = TERRAIN_N_Y-(int)((pn->state[YY]-aa->tt->min[YY])/DISTANCE)-1;
        DrawFoot(show_terrain, position, (3.1416f/2.0f - pn->state[ANGLE]));
#endif

        foot_sequence.x[count] = pn->state[XX];
        foot_sequence.y[count] = pn->state[YY];	
        foot_sequence.theta[count] = pn->state[ANGLE];
        foot_sequence.s[count] = pn->state[SIDE];	
        foot_sequence.height[count] = pn->state[ZZ];
	
	int index_x, index_y;
	terrain_indices( aa->tt, pn->state[XX], pn->state[YY], &index_x, &index_y, NULL);

        foot_sequence.normal_x[count] = normal_value[index_x][index_y][0];
	foot_sequence.normal_y[count] = normal_value[index_x][index_y][1];
	foot_sequence.normal_z[count] = normal_value[index_x][index_y][2];

        pn = pn->previous;
        count++;	
        if (count > MAX_FOOT_NO-2)
          break;	
      }
    }

    if (foot_sequence.s[count-1] == LEFT){
      foot_sequence.x[count] = foot_sequence.x[count-1] + HIP_WIDTH*sinf(foot_sequence.theta[count-1]);
      foot_sequence.y[count] = foot_sequence.y[count-1] - HIP_WIDTH*cosf(foot_sequence.theta[count-1]);
      foot_sequence.s[count] = RIGHT;
    }
    else{
      foot_sequence.x[count] = foot_sequence.x[count-1] - HIP_WIDTH*sinf(foot_sequence.theta[count-1]);
      foot_sequence.y[count] = foot_sequence.y[count-1] + HIP_WIDTH*cosf(foot_sequence.theta[count-1]);
      foot_sequence.s[count] = LEFT;
    }
    foot_sequence.theta[count] = foot_sequence.theta[count-1];
    foot_sequence.height[count] = foot_sequence.height[count-1];

    foot_sequence.normal_x[count] = foot_sequence.normal_x[count-1];
    foot_sequence.normal_y[count] = foot_sequence.normal_y[count-1];
    foot_sequence.normal_z[count] = foot_sequence.normal_z[count-1];


    foot_sequence.header.stamp = ros::Time::now();
    foot_sequence_pub_.publish(foot_sequence);

#ifdef DRAW
    cvShowImage("depth_map", show_terrain);	
#endif		
  }	


void
compute_surface_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));

  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);

  // Set the input points
  norm_est.setInputCloud (points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}


/************************Call back function list **************************/

void Goal_posCallback(const interactive_walk_planner::goal_pose::ConstPtr& msg)
{
  if (msg->header.stamp.toSec() < initTime.toSec()) {
    printf("\n\n\n\nbogus message, %g %g\n\n\n\n\n", msg->header.stamp.toSec(), initTime.toSec());
  }
  else {
	  goal_pos_x = msg->x;
	  goal_pos_y = msg->y;
          plan_method = msg->plan;
          printf("Goal message received, %g %g Planning method %d\n", goal_pos_x, goal_pos_y, plan_method);
	  GoalposeUpdated_ = true;	
  }
}

void ReplanCallback(const interactive_walk_planner::replan::ConstPtr& msg)
{
  if (msg->header.stamp.toSec() < initTime.toSec()) {
    printf("\n\n\n\nbogus message, %g %g\n\n\n\n\n", msg->header.stamp.toSec(), initTime.toSec());
  }
  else {
	  NoStopMapping_ = true;	
	  GoalposeUpdated_ = false;
	  printf("\n\n\n\n Please Resend the new goal location\n\n\n\n\n"); 		
  }
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>); 

  sensor_msgs::PointCloud2 Rotated_Input, Rotated_Input2, Rotated_Input3;
  Eigen::Matrix4f tform, tform2, tform3;
  tform.setIdentity();
  tform2.setIdentity();
  tform3.setIdentity();
  float theta = 1.0*M_PI/180.0;
  float theta2 = 4.0*M_PI/180.0;
  float theta3 = -3.0*M_PI/180.0;
  Eigen::Matrix3f rotation, rotation2, rotation3;
  rotation << 1, 0, 0, 0, cosf(theta), -sinf(theta), 0, sinf(theta), cosf(theta);
  rotation2 << cosf(theta2), 0, sinf(theta2), 0, 1, 0, -sinf(theta2), 0, cosf(theta2);
  rotation3 << cosf(theta3), -sinf(theta3), 0, sinf(theta3), cosf(theta2), 0, 0, 0, 1;
  tform.topLeftCorner (3, 3) = rotation;
  tform2.topLeftCorner (3, 3) = rotation2;
  tform3.topLeftCorner (3, 3) = rotation3;
  pcl_ros::transformPointCloud(tform, *cloud, Rotated_Input);
  pcl_ros::transformPointCloud(tform2, Rotated_Input, Rotated_Input2);
  pcl_ros::transformPointCloud(tform3, Rotated_Input2, Rotated_Input3);

  pcl::fromROSMsg (Rotated_Input3, *cloud2);
  	
  //pcl::fromROSMsg (*cloud, *cloud2);

  pthread_mutex_lock(&mtx_);
  pcl::copyPointCloud(*cloud2, *cloud_sc_);
  pthread_mutex_unlock(&mtx_);
  //std::cout << "cloud received"<< std::endl;
  NewclouddataRefresh_ = true;

}

/************************Thread function list **************************/

void visualizerFunc()
{


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters();

	while(!viewer->wasStopped())
	{
		// Refresh data if necessary
		if (viewerNeedsRefresh_)
		{
		  viewerNeedsRefresh_ = false;
		  const std::string pc_id = "cloud";

		      viewer->removePointCloud("cloud");
			  pthread_mutex_lock(&refreshMtx_);
			  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_filtered, normals1, 10, 0.05, pc_id);
			  pthread_mutex_unlock(&refreshMtx_);


		}

		viewer->spinOnce(1);
		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}
}

void mapbulidFunc()
{
  static int count = 0;
  int x_index, y_index, input;
  clock_t start, end;

  float terrain[TERRAIN_N_X][TERRAIN_N_Y];
  float normal[TERRAIN_N_X][TERRAIN_N_Y][3];
  double terrain_count[TERRAIN_N_X][TERRAIN_N_Y] = {{0}};

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter3 (new pcl::PointCloud<pcl::PointXYZ>);
  while(1){
    if (NewclouddataRefresh_&&NoStopMapping_) {
      start = clock();	
      //std::cout << "start map build"<< std::endl; 
      NewclouddataRefresh_ = false;
      
      pthread_mutex_lock(&mtx_);
      pcl::copyPointCloud(*cloud_sc_, *cloud_old);
      pthread_mutex_unlock(&mtx_);

	pcl::PassThrough<pcl::PointXYZ> pass_out;
	pass_out.setInputCloud (cloud_old);
	pass_out.setFilterFieldName ("x");
	pass_out.setFilterLimits (0.20, 1.5);
	pass_out.filter (*cloud_filter1);
	pass_out.setInputCloud (cloud_filter1);
	pass_out.setFilterFieldName ("y");
	pass_out.setFilterLimits (-1.5, 1.5);
	pass_out.filter (*cloud_filter2);
	pass_out.setInputCloud (cloud_filter2);	
	pass_out.setFilterFieldName ("z");
	pass_out.setFilterLimits (-0.25, 0.25);
	pass_out.filter (*cloud_filter3);

	sensor_msgs::PointCloud2 Filtered_cloud;
	pcl::toROSMsg(*cloud_filter3, Filtered_cloud);
	Filtered_cloud.header.stamp = ros::Time::now();
	output_pub_.publish(Filtered_cloud);  


	pthread_mutex_lock(&refreshMtx_);
	pcl::copyPointCloud(*cloud_filter3, *cloud_filtered);
	compute_surface_normals (cloud_filtered, 0.1, normals1);
	viewerNeedsRefresh_ = true;
	pthread_mutex_unlock(&refreshMtx_);

      pcl::copyPointCloud(*cloud_filter3, *cloud);

      for (int i=0;i<TERRAIN_N_X;i++) {
        for (int j=0;j<TERRAIN_N_Y;j++){
          terrain[i][j] = 0;
          terrain_count[i][j] = 0.0;
	  normal[i][j][0] = 0.0;
	  normal[i][j][1] = 0.0;
	  normal[i][j][2] = 1.0;			
        }
      }	

      for(int i=0; i<cloud->points.size();i++){

	if (cloud->points[i].x>-0.1 && cloud->points[i].x < 0.3 && cloud->points[i].y >-0.4 && cloud->points[i].y <0.1 && cloud->points[i].z>0.05)
		continue;

        if (cloud->points[i].x+0.0<MIN_X)
          x_index = 0;
        else if(cloud->points[i].x+0.0>MAX_X||cloud->points[i].x+0.0==MAX_X)
          x_index = TERRAIN_N_X-1;
        else
          x_index = (int)((cloud->points[i].x-MIN_X+0.0)/DISTANCE);	
        if (cloud->points[i].y<MIN_Y)
          y_index = 0;
        else if(cloud->points[i].y>MAX_Y||cloud->points[i].y==MAX_Y)
          y_index = TERRAIN_N_Y-1;
        else
          y_index = (int)((cloud->points[i].y-MIN_Y)/DISTANCE);
	
        terrain[x_index][y_index] += cloud->points[i].z+0.0;
        terrain_count[x_index][y_index] =terrain_count[x_index][y_index] + 1.0;

        normal[x_index][y_index][0] += normals1->at(i).normal_x;
	normal[x_index][y_index][1] += normals1->at(i).normal_y;
	normal[x_index][y_index][2] += normals1->at(i).normal_z;

      }

      for (int i=0; i<TERRAIN_N_X;i++)
      {
        for(int j=0; j<TERRAIN_N_Y;j++)
        {
          if (terrain_count[i][j] != 0){
            terrain[i][j] = terrain[i][j]/terrain_count[i][j];
	    normal[i][j][0] = normal[i][j][0]/terrain_count[i][j];
	    normal[i][j][1] = normal[i][j][1]/terrain_count[i][j];
	    normal[i][j][2] = normal[i][j][2]/terrain_count[i][j];
		
	  }
        }
      }
      // plot the terrain
      for(int i=0; i<TERRAIN_N_X; i++){
        for (int j=0; j<TERRAIN_N_Y; j++){	
 	    float image_height = 0;	
            if (terrain[i][j]<0)
		image_height = 0;
            else
            	image_height = terrain[i][j];    	
            if (image_height > (0 + 0.5))
              image_height = (0 + 0.5);	
            uchar *ptr4=(uchar *)cvPtr2D(local_terrain,TERRAIN_N_Y-j-1,i);
            ptr4[0]=(int)(255*(0-image_height+0.5)/0.5f);
            ptr4[1]=(int)(255*(0-image_height+0.5)/0.5f);
            ptr4[2]=(int)(255*(0-image_height+0.5)/0.5f);	
        }
      }

      // pass the terrain information to path planner
      pthread_mutex_lock(&terrain_mutex);
      for(int i=0; i<TERRAIN_N_X; i++){
        for (int j=0; j<TERRAIN_N_Y; j++){
          terrain_value[i][j] = terrain[i][j];
	  normal_value[i][j][0] = normal[i][j][0];
	  normal_value[i][j][1] = normal[i][j][1];
	  normal_value[i][j][2] = normal[i][j][2];		
        }
      }
      pthread_mutex_unlock(&terrain_mutex); 

      Mat block_image;
      block_image = cv::Mat(local_terrain,true);
#ifdef DRAW      		
      imshow( image_window, block_image);
#endif 	
      std::cout << "point cloud data process cycle: " << count << std::endl; 	
	
      if (GoalposeUpdated_ && plan_method ==1){	
          	planNeedsRefresh_ = true;
		AlignNeedsRefresh_ = false;
		NoStopMapping_ = false;
      }
      else if(GoalposeUpdated_ && plan_method ==2){	
		planNeedsRefresh_ = false;
		AlignNeedsRefresh_ = true;		
		NoStopMapping_ = false;
                
	        Point2f block_center;
	        double fDir;
	        block_center.x = round((goal_pos_x+2.0)/0.02);
	        block_center.y = round((2.0-goal_pos_y)/0.02);	
		Mat origin_image;
		cvtColor(block_image, origin_image, CV_BGR2GRAY);
		imwrite( "/home/huangwei/Test_Image.jpg", origin_image );
		printf("rough_center.x = %.2f, rough_center.y = %.2f, \n", block_center.x, block_center.y);
	        if( getPose2DFast(origin_image, block_center, &fDir)==0)
	        { 
	        	printf("success.\n block_center.x = %.2f, block_center.y = %.2f, dir = %.2f\n", block_center.x, block_center.y, fDir);


			while (fDir>=90 || fDir<0){
				if (fDir>=90)
					fDir = fDir-90;
				if (fDir<0)
					fDir = fDir+90;
			} 	
			
			if (fDir>45)
				fDir = fDir-90;		

			fDir = fDir*CV_PI/180.;
			Point2f block_center_f, foot_center, left_foot, right_foot;

			block_center_f.x = block_center.x*0.02-2.0;
			block_center_f.y = 2.0 - block_center.y*0.02;

			foot_center.x = block_center_f.x - 0.40*cosf(-fDir);
  			foot_center.y = block_center_f.y - 0.40*sinf(-fDir);
			printf("foot_center.x = %.2f, foot_center.y = %.2f\n", foot_center.x, foot_center.y);

			left_foot.x = foot_center.x - 0.13*sinf(-fDir);
			left_foot.y = foot_center.y + 0.13*cosf(-fDir);    
			right_foot.x = foot_center.x + 0.13*sinf(-fDir);
			right_foot.y = foot_center.y - 0.13*cosf(-fDir); 

			interactive_walk_planner::foot_sequence foot_sequence;



			float max_x_step_length = 0.2;
			float max_y_step_length = 0.05;


			int x_step_no = ceil(fabsf(left_foot.x-0.045)/max_x_step_length);
			int y_step_no = ceil(fabsf(left_foot.y)/max_y_step_length);
			int max_step_no;
			if (x_step_no>y_step_no)
				max_step_no = x_step_no;
			else
				max_step_no = y_step_no;


			foot_sequence.x[0] = 0.045;
			foot_sequence.y[0] = 0;	
			foot_sequence.theta[0] = 0;
			foot_sequence.s[0] = 0;	
			foot_sequence.height[0] = 0.0;

			foot_sequence.normal_x[0] = 0;
			foot_sequence.normal_y[0] = 0;
			foot_sequence.normal_z[0] = 1;


			for (int i=0; i<max_step_no; i++){

				foot_sequence.x[2*i+1] = 0.045+(i+1)*(left_foot.x-0.045)/max_step_no;
				foot_sequence.y[2*i+1] = -0.26+(i+1)*(left_foot.y)/max_step_no;	
				foot_sequence.theta[2*i+1] = (-fDir)/max_step_no;
				foot_sequence.s[2*i+1] = 1;	
				foot_sequence.height[2*i+1] = 0.0;

				foot_sequence.normal_x[2*i+1] = 0;
				foot_sequence.normal_y[2*i+1] = 0;
				foot_sequence.normal_z[2*i+1] = 1;

				foot_sequence.x[2*i+2] = 0.045+(i+1)*(left_foot.x-0.045)/max_step_no;
				foot_sequence.y[2*i+2] = 0+(i+1)*(left_foot.y)/max_step_no;	
				foot_sequence.theta[2*i+2] = (-fDir)/max_step_no;
				foot_sequence.s[2*i+2] = 0;	
				foot_sequence.height[2*i+2] = 0.0;

				foot_sequence.normal_x[2*i+2] = 0;
				foot_sequence.normal_y[2*i+2] = 0;
				foot_sequence.normal_z[2*i+2] = 1;
			}			


			foot_sequence.x[2*max_step_no+1] = right_foot.x;
			foot_sequence.y[2*max_step_no+1] = right_foot.y;	
			foot_sequence.theta[2*max_step_no+1] = -fDir;
			foot_sequence.s[2*max_step_no+1] = 1;	
			foot_sequence.height[2*max_step_no+1] = 0.0;

			foot_sequence.normal_x[2*max_step_no+1] = 0;
			foot_sequence.normal_y[2*max_step_no+1] = 0;
			foot_sequence.normal_z[2*max_step_no+1] = 1;

			foot_sequence.x[2*max_step_no+2] = left_foot.x;
			foot_sequence.y[2*max_step_no+2] = left_foot.y;	
			foot_sequence.theta[2*max_step_no+2] = -fDir;
			foot_sequence.s[2*max_step_no+2] = 0;	
			foot_sequence.height[2*max_step_no+2] = 0.0;

			foot_sequence.normal_x[2*max_step_no+2] = 0;
			foot_sequence.normal_y[2*max_step_no+2] = 0;
			foot_sequence.normal_z[2*max_step_no+2] = 1;

		        foot_sequence.header.stamp = ros::Time::now();
			foot_sequence_pub_.publish(foot_sequence);
			

	        } 
      }		
        count++;
        end = clock();
      }
    }
  }

  void pathplanFunc()
  {
    int i, j, input1, index;
    clock_t start, end;
    float normal[TERRAIN_N_X][TERRAIN_N_Y][3];	

    //creat the terrain
    t1 = create_terrain();
	
    // Find the path based on Astar	
    while(1){

      if (planNeedsRefresh_&&GoalposeUpdated_&&path_done==0){
        start = clock();
        planNeedsRefresh_ = false;

        pthread_mutex_lock(&terrain_mutex);
        t1->min[XX] = MIN_X;
        t1->min[YY] = MIN_Y;
        t1->max[XX] = MAX_X;
        t1->max[YY] = MAX_Y;
        // See how far away perception horizon is.
        t1->perception_radius = 2*TERRAIN_N_Y;
        generate_true_cost_map( t1, terrain_value);
        cvCopy(local_terrain, show_terrain);
	      for(int i=0; i<TERRAIN_N_X; i++){
		for (int j=0; j<TERRAIN_N_Y; j++){
		  curr_normal_value[i][j][0] = normal_value[i][j][0];
		  curr_normal_value[i][j][1] = normal_value[i][j][1];
		  curr_normal_value[i][j][2] = normal_value[i][j][2];		
		}
	      }

        pthread_mutex_unlock(&terrain_mutex);

        // set current foot and goal location
        t1->current[XX] = 0.045;
        t1->current[YY] = 0.0;
        t1->current[ANGLE] = 0; 
        t1->current[SIDE] = LEFT; 

        t1->goal[XX] = goal_pos_x;
        t1->goal[YY] = goal_pos_y;
        t1->goal[ANGLE] = 0;
        t1->goal[SIDE] = LEFT; 

        std::cout << "initialize current foot pose " << t1->current[XX] <<" " << t1->current[YY] <<" " << t1->current[ANGLE] <<" " << std::endl;
        std::cout << "initialize goal foot pose " << t1->goal[XX] <<" " << t1->goal[YY] <<" " << t1->goal[ANGLE] <<" " << std::endl;

        //generate the foot sequence by A*
        a1 = create_astar( t1 );		
        plan();
        if (p1!=NULL){
              printf_final_path( p1 );	
	      DrawFootSequence (a1, p1);
	      //std::cout << "Planner stop here, replanning is under developing " << std::endl;	
	      //std::cin >> input1;		
         }
         p1 = NULL;
         end = clock();
         std::cout << "One step planning time is" << (end-start)/(1000) << " ms" << std::endl;
      }	  			
    }
  }

  /*************************************************************************/

  int main(int argc, char **argv)
  {
    signal(SIGINT,quit);
    ros::init(argc, argv, "map_build", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    std::cout << "system starts" << std::endl;

    initTime = ros::Time::now();
    if (!nh_param.getParam("action_path", ActionPath))
	{
		ROS_WARN("No action set loaded");
	}
    ROS_INFO("Action path is: %s",ActionPath.c_str());

#ifdef DRAW
    cvNamedWindow("depth_map",CV_WINDOW_NORMAL);
    namedWindow( image_window, CV_WINDOW_NORMAL);
    cvStartWindowThread();
    image_transport::ImageTransport it(nh);
#endif
    

    ros::Subscriber sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/assembled_cloud_ocu", 100, &cloud_cb);
    //ros::Subscriber sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_cloud", 100, &cloud_cb);

    ros::Subscriber goal_pos_ = nh.subscribe<interactive_walk_planner::goal_pose>("/interactive_walk_planner/goal_pose", 100, &Goal_posCallback);
    ros::Subscriber replan_message_ = nh.subscribe<interactive_walk_planner::replan>("/interactive_walk_planner/replan_message", 100, &ReplanCallback);
    foot_sequence_pub_ = nh.advertise<interactive_walk_planner::foot_sequence>("/atlas/planned_foot_sequence", 100, true );	
    output_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 100);


    NewclouddataRefresh_ = false;
    NoStopMapping_ = true;	
    planNeedsRefresh_ = false;
    AlignNeedsRefresh_ = false;
    GoalposeUpdated_ = false;
    viewerNeedsRefresh_ = false;
 

    boost::thread mapbuildThread(mapbulidFunc);
    boost::thread pathplanThread(pathplanFunc);
#ifdef NORMAL
    boost::thread visualizerThread(visualizerFunc);
#endif
    boost::thread_group group;
    group.add_thread(&mapbuildThread);
    group.add_thread(&pathplanThread);
#ifdef NORMAL
    group.add_thread(&visualizerThread);
#endif
#ifdef DRAW
    // Initilize the map matrix
    assert(local_terrain);
#endif
    //run callbacks
    ros::spin();
    group.join_all();

  }

