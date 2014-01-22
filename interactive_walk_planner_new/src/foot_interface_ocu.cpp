#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <interactive_walk_planner_new/goal_pose.h>
#include <interactive_walk_planner_new/task_mode.h>
#include <interactive_walk_planner_new/replan.h>
#include <interactive_walk_planner_new/foot_sequence.h>
#include <interactive_walk_planner_new/orientation.h>

// for offline stepping
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>

#include <atlas_ros_msgs/AtlasStepFeedback.h>
#include <atlas_ros_msgs/AtlasStepParams.h>
#include <atlas_ros_msgs/AtlasWalkParams.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>

#define MAX_FOOT_NO 50

using namespace visualization_msgs;
static ros::Publisher goal_location_pub_;
static ros::Publisher task_mode_pub_;
static ros::Publisher pub_wrecs_step;
static ros::Publisher replan_message_pub_;
static int longest_marker_no=0;
static int task_mode = 0; // 0 ramp 1 zig_zag 2 walk up cinder block 3 walk down cinder block

void makeMovingMarker( geometry_msgs::Pose position, std::string markerName, bool goal, bool foot_side);

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%

float goal_pos[2] = {0.0, 0.0};
float ori_estimation = 0;
float foot_sequence[MAX_FOOT_NO][8];
volatile bool foot_sequence_update;
ros::Time initTime;

static void sendStep(double stepX, double stepY, double stepZ, double swing_height, double step_yaw, int foot_index, double duration, int index) {
        atlas_ros_msgs::AtlasStepData step;
        step.step_index = index;
        step.foot_index = foot_index;
        step.duration = duration;
        step.position.x = stepX;
        step.position.y = stepY;
        step.position.z = stepZ;
        step.yaw = step_yaw;
        step.normal.x = 0.0;
        step.normal.y = 0.0;
        step.normal.z = 1.0;
        step.swing_height = swing_height;
        pub_wrecs_step.publish(step);

        usleep(30000);
}
// %Tag(Box)%
Marker makeBox( bool goal, bool Rfoot)
{
  Marker marker;
  if (goal){
        marker.type = Marker::SPHERE;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
          marker.color.r = 1.0;
  }
  else{
        marker.type = Marker::CUBE;
        marker.scale.x = 0.25;
        marker.scale.y = 0.13;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
  }        
  if (Rfoot){        
          marker.color.g = 1.0;
          marker.color.b = 0.0;
  }
  else{
          marker.color.g = 0.0;
          marker.color.b = 1.0;
  }        
  marker.color.a = 1.0;

  return marker;
}


Marker makeManuBox()
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.05;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;        
  marker.color.a = 1.0;

  return marker;
}
// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  switch ( feedback->event_type )
  {

      case InteractiveMarkerFeedback::MENU_SELECT:

                if(feedback->menu_entry_id ==1){


                        // piblish the replan command
                        interactive_walk_planner_new::replan msg;
                        msg.x = 1;
                        msg.header.stamp = ros::Time::now();
                        replan_message_pub_.publish(msg);

                        for (int i=0; i<MAX_FOOT_NO; i++){
                                for (int j=0; j<7; j++)
                                         foot_sequence[i][j] = 0;
                                foot_sequence[i][7] = 1.0;
                        }
                        printf( "The current step lenght is %d \n", longest_marker_no);
                        for (int j=1; j<longest_marker_no; j++){
                         std::stringstream ss;
                         ss << j+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = 100;
                         position.position.y = 100;
                         position.position.z = 0;
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = 0;
                         position.orientation.w = 0;
                         makeMovingMarker(position, s, false, true);
                        }
                        server->applyChanges();        
                        ROS_INFO_STREAM( ": Replan command published" );

                }
                else if (feedback->menu_entry_id ==3){
                        // piblish the goal location
                        interactive_walk_planner_new::goal_pose goal_location;
                        goal_location.header.stamp = ros::Time::now();
                        goal_location.x = goal_pos[0];         
                        goal_location.y = goal_pos[1];
                        goal_location.plan = 1;
                        goal_location_pub_.publish(goal_location);
                        ROS_INFO_STREAM( ": A* based search is used with new goal location" );

                }
                else if (feedback->menu_entry_id ==4){
                        // piblish the goal location
                        interactive_walk_planner_new::goal_pose goal_location;
                        goal_location.header.stamp = ros::Time::now();
                        goal_location.x = goal_pos[0];         
                        goal_location.y = goal_pos[1];
                        goal_location.plan = 2;
                        goal_location_pub_.publish(goal_location);
                        ROS_INFO_STREAM( ": Alian with target cinder block is used with new goal location" );

                }
                else if (feedback->menu_entry_id ==6){

                        for (int i=0; i<MAX_FOOT_NO; i++){
                         foot_sequence[0][7] = 0.045;
                         for (int j=1; j<7; j++)        
                                 foot_sequence[i][j] = 0;
                         foot_sequence[i][7] = 1.0;        
                        
                         if (i==1){
                                 foot_sequence[1][0] = 0.30;
                                 foot_sequence[1][1] = 0.0;
                                 foot_sequence[1][2] = 0;
                                 foot_sequence[1][3] = 0;
                                 foot_sequence[1][4] = 0.16;
                                foot_sequence[1][5] = 0.0; //normal_x
                         foot_sequence[1][6] = 0.0; // normal_y
                                foot_sequence[1][7] = 1.0; //normal_z
                                 printf( "The step 1 position is %g %g %g \n", foot_sequence[1][0], foot_sequence[1][1], foot_sequence[1][4]);
                         }
                         else if (i==2){
                                 foot_sequence[2][0] = 0.30;
                                 foot_sequence[2][1] = -0.20;
                                 foot_sequence[2][2] = 0;
                                 foot_sequence[2][3] = 1;
                                 foot_sequence[2][4] = 0.16;
                                foot_sequence[2][5] = 0.0; //normal_x
                         foot_sequence[2][6] = 0.0; // normal_y
                                foot_sequence[2][7] = 1.0; //normal_z
                                 printf( "The step 2 position is %g %g %g \n", foot_sequence[2][0], foot_sequence[2][1], foot_sequence[2][4]);
                         }
                 
                         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                                        if (longest_marker_no < i)
                                                longest_marker_no = i;
                                        else{
                                                for (int j=i; j<longest_marker_no; j++){
                                                 std::stringstream ss;
                                                 ss << j+1;
                                                 std::string s = ss.str();
                                                 geometry_msgs::Pose position;
                                                 position.position.x = 100;
                                                 position.position.y = 100;
                                                 position.position.z = 0;
                                                 position.orientation.x = 0;
                                                 position.orientation.y = 0;
                                                 position.orientation.z = 0;
                                                 position.orientation.w = 0;
                                                 makeMovingMarker(position, s, false, true);
                                                }
                                        }
                                        break;        
                 }
                         std::stringstream ss;
                         ss << i+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = foot_sequence[i][0];
                         position.position.y = foot_sequence[i][1];
                         position.position.z = foot_sequence[i][4];

                            float n_x = foot_sequence[i][5];
                              float n_y = foot_sequence[i][6];
                         float n_z = foot_sequence[i][7];        

                         position.orientation.w = 1;        
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = 0;
                        
                         if (foot_sequence[i][3] > 0)
                                 makeMovingMarker(position, s, false, true);
                         else
                                makeMovingMarker(position, s, false, false);

                        }
                          server->applyChanges();        
                        ROS_INFO_STREAM( ": Plan the walk up cinder block sequence" );
                }
                else if (feedback->menu_entry_id ==7){

                        for (int i=0; i<MAX_FOOT_NO; i++){
                         for (int j=0; j<7; j++)                                
                                 foot_sequence[i][j] = 0;                                        
                         foot_sequence[0][7] = 1;

                         if (i==1){
                                 foot_sequence[1][0] = 0.30;
                                 foot_sequence[1][1] = 0.00;
                                 foot_sequence[1][2] = 0;
                                 foot_sequence[1][3] = 0;
                                 foot_sequence[1][4] = -0.16;
                                foot_sequence[1][5] = 0.0; //normal_x
                         foot_sequence[1][6] = 0.0; // normal_y
                                foot_sequence[1][7] = 1.0; //normal_z
                                 printf( "The step 1 position is %g %g %g \n", foot_sequence[1][0], foot_sequence[1][1], foot_sequence[1][4]);
                         }
                         else if (i==2){
                                 foot_sequence[2][0] = 0.30;
                                 foot_sequence[2][1] = -0.20;
                                 foot_sequence[2][2] = 0;
                                 foot_sequence[2][3] = 1;
                                 foot_sequence[2][4] = -0.16;
                                foot_sequence[2][5] = 0.0; //normal_x
                         foot_sequence[2][6] = 0.0; // normal_y
                                foot_sequence[2][7] = 1.0; //normal_z
                                 printf( "The step 2 position is %g %g %g \n", foot_sequence[2][0], foot_sequence[2][1], foot_sequence[3][4]);
                         }
           
                         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                                        if (longest_marker_no < i)
                                                longest_marker_no = i;
                                        else{
                                                for (int j=i; j<longest_marker_no; j++){
                                                 std::stringstream ss;
                                                 ss << j+1;
                                                 std::string s = ss.str();
                                                 geometry_msgs::Pose position;
                                                 position.position.x = 100;
                                                 position.position.y = 100;
                                                 position.position.z = 0;
                                                 position.orientation.x = 0;
                                                 position.orientation.y = 0;
                                                 position.orientation.z = 0;
                                                 position.orientation.w = 0;
                                                 makeMovingMarker(position, s, false, true);
                                                }
                                        }
                                        break;        
                 }
                         std::stringstream ss;
                         ss << i+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = foot_sequence[i][0];
                         position.position.y = foot_sequence[i][1];
                         position.position.z = foot_sequence[i][4];

                            float n_x = foot_sequence[i][5];
                              float n_y = foot_sequence[i][6];
                         float n_z = foot_sequence[i][7];        

                         position.orientation.w = 1;        
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = 0;
                         if (foot_sequence[i][3] > 0)
                                 makeMovingMarker(position, s, false, true);
                         else
                                makeMovingMarker(position, s, false, false);

                        }
                          server->applyChanges();        
                        ROS_INFO_STREAM( ": Send the walk down cinder block sequence" );
                }                        
                else if (feedback->menu_entry_id ==9){

                        for (int i=0; i<MAX_FOOT_NO; i++){
                         for (int j=0; j<7; j++)        
                                 foot_sequence[i][j] = 0;                
                         foot_sequence[0][0] = 0.045;                        
                         foot_sequence[0][7] = 1;
                         if (i==1){
                                  foot_sequence[1][0] = 0.095;
                                  foot_sequence[1][1] = -0.26;
                                  foot_sequence[1][2] = 0;
                                  foot_sequence[1][3] = 1;
                                  foot_sequence[1][4] = 0.0;
                                 foot_sequence[1][5] = 0.0; //normal_x
                                 foot_sequence[1][6] = 0.0; // normal_y
                                 foot_sequence[1][7] = 1.0; //normal_z
                                  printf( "The step 1 position is %g %g %g \n", foot_sequence[1][0], foot_sequence[1][1], foot_sequence[1][4]);
                         }
                         if (i==2){
                                  foot_sequence[2][0] = 0.095;
                                  foot_sequence[2][1] = 0.0;
                                  foot_sequence[2][2] = 0;
                                  foot_sequence[2][3] = 0;
                                  foot_sequence[2][4] = 0.0;
                                 foot_sequence[2][5] = 0.0; //normal_x
                                 foot_sequence[2][6] = 0.0; // normal_y
                                 foot_sequence[2][7] = 1.0; //normal_z
                                  printf( "The step 2 position is %g %g %g \n", foot_sequence[2][0], foot_sequence[2][1], foot_sequence[3][4]);
                         }        
        
                         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                                        if (longest_marker_no < i)
                                                longest_marker_no = i;
                                        else{
                                                for (int j=i; j<longest_marker_no; j++){
                                                 std::stringstream ss;
                                                 ss << j+1;
                                                 std::string s = ss.str();
                                                 geometry_msgs::Pose position;
                                                 position.position.x = 100;
                                                 position.position.y = 100;
                                                 position.position.z = 0;
                                                 position.orientation.x = 0;
                                                 position.orientation.y = 0;
                                                 position.orientation.z = 0;
                                                 position.orientation.w = 0;
                                                 makeMovingMarker(position, s, false, true);
                                                }
                                        }
                                        break;        
                 }
                         std::stringstream ss;
                         ss << i+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = foot_sequence[i][0];
                         position.position.y = foot_sequence[i][1];
                         position.position.z = foot_sequence[i][4];

                            float n_x = foot_sequence[i][5];
                              float n_y = foot_sequence[i][6];
                         float n_z = foot_sequence[i][7];        

                         position.orientation.w = cosf(foot_sequence[i][2]/2.);        
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = sinf(foot_sequence[i][2]/2.);
                         if (foot_sequence[i][3] > 0)
                                 makeMovingMarker(position, s, false, true);
                         else
                                makeMovingMarker(position, s, false, false);

                        }
                          server->applyChanges();        
                        ROS_INFO_STREAM( ": print the small walking forward sequence" );
                }
                else if (feedback->menu_entry_id ==10){

                        for (int i=0; i<MAX_FOOT_NO; i++){
                         for (int j=0; j<7; j++)        
                                 foot_sequence[i][j] = 0;                
                         foot_sequence[0][0] = 0.045;                        
                         foot_sequence[0][7] = 1;
                         if (i==1){
                                  foot_sequence[1][0] = 0.095;
                                  foot_sequence[1][1] =0.05;
                                  foot_sequence[1][2] = 0.0;
                                  foot_sequence[1][3] = 0;
                                  foot_sequence[1][4] = 0.0;
                                 foot_sequence[1][5] = 0.0; //normal_x
                                 foot_sequence[1][6] = 0.0; // normal_y
                                 foot_sequence[1][7] = 1.0; //normal_z
                                  printf( "The step 1 position is %g %g %g \n", foot_sequence[1][0], foot_sequence[1][1], foot_sequence[1][4]);
                         }
                         if (i==2){        
                                  foot_sequence[2][0] = 0.095;
                                  foot_sequence[2][1] = -0.20;
                                  foot_sequence[2][2] = 0.0;
                                  foot_sequence[2][3] = 1;
                                  foot_sequence[2][4] = 0.0;
                                 foot_sequence[2][5] = 0.0; //normal_x
                                 foot_sequence[2][6] = 0.0; // normal_y
                                 foot_sequence[2][7] = 1.0; //normal_z
                                  printf( "The step 2 position is %g %g %g \n", foot_sequence[2][0], foot_sequence[2][1], foot_sequence[3][4]);
                         }        
        
                         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                                        if (longest_marker_no < i)
                                                longest_marker_no = i;
                                        else{
                                                for (int j=i; j<longest_marker_no; j++){
                                                 std::stringstream ss;
                                                 ss << j+1;
                                                 std::string s = ss.str();
                                                 geometry_msgs::Pose position;
                                                 position.position.x = 100;
                                                 position.position.y = 100;
                                                 position.position.z = 0;
                                                 position.orientation.x = 0;
                                                 position.orientation.y = 0;
                                                 position.orientation.z = 0;
                                                 position.orientation.w = 0;
                                                 makeMovingMarker(position, s, false, true);
                                                }
                                        }
                                        break;        
                 }
                         std::stringstream ss;
                         ss << i+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = foot_sequence[i][0];
                         position.position.y = foot_sequence[i][1];
                         position.position.z = foot_sequence[i][4];

                            float n_x = foot_sequence[i][5];
                              float n_y = foot_sequence[i][6];
                         float n_z = foot_sequence[i][7];        

                         position.orientation.w = cosf(foot_sequence[i][2]/2.);
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = sinf(foot_sequence[i][2]/2.);
                         if (foot_sequence[i][3] > 0)
                                 makeMovingMarker(position, s, false, true);
                         else
                                makeMovingMarker(position, s, false, false);

                        }
                          server->applyChanges();        
                        ROS_INFO_STREAM( ": print the small step left" );
                }
                else if (feedback->menu_entry_id ==11){

                        for (int i=0; i<MAX_FOOT_NO; i++){        
                         for (int j=0; j<7; j++)        
                                 foot_sequence[i][j] = 0;                
                         foot_sequence[0][0] = 0.045;                        
                         foot_sequence[0][7] = 1;
                         if (i==1){
                                  foot_sequence[1][0] = 0.095;
                                  foot_sequence[1][1] = -0.30;
                                  foot_sequence[1][2] = 0.0;
                                  foot_sequence[1][3] = 1;
                                  foot_sequence[1][4] = 0.0;
                                 foot_sequence[1][5] = 0.0; //normal_x
                                 foot_sequence[1][6] = 0.0; // normal_y
                                 foot_sequence[1][7] = 1.0; //normal_z
                                  printf( "The step 1 position is %g %g %g \n", foot_sequence[1][0], foot_sequence[1][1], foot_sequence[1][4]);
                         }
                         if (i==2){        
                                  foot_sequence[2][0] = 0.095;
                                  foot_sequence[2][1] = -0.05;
                                  foot_sequence[2][2] = 0.0;
                                  foot_sequence[2][3] = 0;
                                  foot_sequence[2][4] = 0.0;
                                 foot_sequence[2][5] = 0.0; //normal_x
                                 foot_sequence[2][6] = 0.0; // normal_y
                                 foot_sequence[2][7] = 1.0; //normal_z
                                  printf( "The step 2 position is %g %g %g \n", foot_sequence[2][0], foot_sequence[2][1], foot_sequence[3][4]);
                         }        
        
                         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                                        if (longest_marker_no < i)
                                                longest_marker_no = i;
                                        else{
                                                for (int j=i; j<longest_marker_no; j++){
                                                 std::stringstream ss;
                                                 ss << j+1;
                                                 std::string s = ss.str();
                                                 geometry_msgs::Pose position;
                                                 position.position.x = 100;
                                                 position.position.y = 100;
                                                 position.position.z = 0;
                                                 position.orientation.x = 0;
                                                 position.orientation.y = 0;
                                                 position.orientation.z = 0;
                                                 position.orientation.w = 0;
                                                 makeMovingMarker(position, s, false, true);
                                                }
                                        }
                                        break;        
                 }
                         std::stringstream ss;
                         ss << i+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = foot_sequence[i][0];
                         position.position.y = foot_sequence[i][1];
                         position.position.z = foot_sequence[i][4];

                            float n_x = foot_sequence[i][5];
                              float n_y = foot_sequence[i][6];
                         float n_z = foot_sequence[i][7];        

                         position.orientation.w = cosf(foot_sequence[i][2]/2.);        
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = sinf(foot_sequence[i][2]/2.);
                         if (foot_sequence[i][3] > 0)
                                 makeMovingMarker(position, s, false, true);
                         else
                                makeMovingMarker(position, s, false, false);

                        }
                          server->applyChanges();        
                        ROS_INFO_STREAM( ": print the small step right" );
                }
                else if (feedback->menu_entry_id ==12){

                        int count = 1;
                           float step_compensation;
                        if (task_mode == 0)
                                step_compensation = 0.0;
                        else
                                step_compensation = 0.00;

                        while (foot_sequence[count][0]!=0 || foot_sequence[count][1]!=0 || foot_sequence[count][2]!=0){
                                atlas_ros_msgs::AtlasStepData step;
                                if (count==1)
                                        step.step_index = -1;
                                else
                                        step.step_index = 0;
                                step.foot_index = foot_sequence[count][3];
                                step.position.x = foot_sequence[count][0]-0.045 - step_compensation*count; // this is for foot sequence command given to ankle instead of center of the foot
                                step.position.y = foot_sequence[count][1];
                                step.position.z = foot_sequence[count][4];
                                
                                step.yaw = foot_sequence[count][2];
                                if (foot_sequence[count][4] - foot_sequence[count-1][4] > 0.1){
                                        step.swing_height = 0.1;
                                        step.duration = 3.0;
                                }
                                else if (foot_sequence[count][4] - foot_sequence[count-1][4] < -0.1){
                                        step.swing_height = 0.05;
                                        step.duration = 3.0;
                                }
                                else{
                                        step.swing_height = 0.05;
                                        step.duration = 1.5;
                                }

                                step.normal.x = foot_sequence[count][5];
                                step.normal.y = foot_sequence[count][6];
                                step.normal.z = foot_sequence[count][7];


                                sendStep(step.position.x, step.position.y, step.position.z, step.swing_height, step.yaw, step.foot_index, step.duration, step.step_index);
                                
                                std::cout << "Published foot step " << step.position.x <<" " << step.position.y <<" " << step.position.z << " "<< step.swing_height << " " << step.yaw <<" " << step.foot_index <<" " << step.duration <<" " << step.step_index <<" " << std::endl;
                                count++;
                                usleep(30000);
                        }
        
                        usleep(100000);
                        ROS_INFO_STREAM( ": Path plan sequence published to ATLAS" );
                        //usleep(35000000);
                }

                else if (feedback->menu_entry_id ==14){
                        // piblish the task mode
                        interactive_walk_planner_new::task_mode task;
                        task.header.stamp = ros::Time::now();
                        task.x = 0;         
                        task_mode_pub_.publish(task);
                        task_mode = 0;
                        ROS_INFO_STREAM( "ramp task command" );
                }
                else if (feedback->menu_entry_id ==15){
                        // piblish the task mode
                        interactive_walk_planner_new::task_mode task;
                        task.header.stamp = ros::Time::now();
                        task.x = 1;         
                        task_mode_pub_.publish(task);
                        task_mode = 1;
                        ROS_INFO_STREAM( "zig_zag task command" );
                }
                else if (feedback->menu_entry_id ==16){
                        // piblish the task mode
                        interactive_walk_planner_new::task_mode task;
                        task.header.stamp = ros::Time::now();
                        task.x = 2;         
                        task_mode_pub_.publish(task);
                        task_mode = 2;
                        ROS_INFO_STREAM( "walk up cinder block task command" );
                }
                else if (feedback->menu_entry_id ==17){
                        // piblish the task mode
                        interactive_walk_planner_new::task_mode task;
                        task.header.stamp = ros::Time::now();
                        task.x = 3;         
                        task_mode_pub_.publish(task);
                        task_mode = 3;
                        ROS_INFO_STREAM( "walk down cinder block task command" );
                }

                break;
      case InteractiveMarkerFeedback::POSE_UPDATE:
                
              std::stringstream ss;
              ss << feedback->marker_name;
               std::string sss;
              ss >> sss;
         int id = std::atoi(sss.c_str());
         if (sss == "goal_pos"){
                goal_pos[0] = feedback->pose.position.x;
                goal_pos[1] = feedback->pose.position.y;
         }        
               else if(id>0){
                foot_sequence[id-1][2] = atan2(2*(feedback->pose.orientation.w*feedback->pose.orientation.z + feedback->pose.orientation.x*feedback->pose.orientation.y), 1-2*(feedback->pose.orientation.y*feedback->pose.orientation.y+feedback->pose.orientation.z*feedback->pose.orientation.z));

         foot_sequence[id-1][0] = feedback->pose.position.x;
         foot_sequence[id-1][1] = feedback->pose.position.y;

                
         ROS_INFO_STREAM( ": Foot step " << id-1 << "Have been changed to" << feedback->pose.position.x << "and" << feedback->pose.position.y );
               }                 
         break;
      }                                                                                

}
// %EndTag(processFeedback)%


////////////////////////////////////////////////////////////////////////////////////

// %Tag(Moving)%
void makeMovingMarker_Goal()
{

  geometry_msgs::Pose position;
      position.position.x = 0.3;
      position.position.y = 0;
      position.position.z = 0.2;
      position.orientation.x = 0;
      position.orientation.y = 0;
      position.orientation.z = 0;
      position.orientation.w = 1;

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/l_foot";
  int_marker.scale = 0.2;
  int_marker.name = "goal_pos";
  int_marker.description = "goal_pos";
  int_marker.pose = position;

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(true, false) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(Moving)%
void makeMovingMarker( geometry_msgs::Pose position, std::string markerName, bool goal, bool foot_side)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/l_foot";
  int_marker.scale = 0.3;
  int_marker.name = markerName;
  int_marker.description = markerName;
  int_marker.pose = position;

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(goal, foot_side) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

// %Tag(Menu)%
void makeMenuMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/l_foot";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "Command Menu";
  int_marker.description = "Command Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeManuBox();
  control.markers.push_back( marker );
  //control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply( *server, int_marker.name );
}
// %EndTag(Menu)%
/*
void ori_estimationCallback(const interactive_walk_planner_new::orientation::ConstPtr& msg)
{

if (msg->header.stamp.toSec() < initTime.toSec()) {
printf("\n\n\n\nbogus message, %g %g\n\n\n\n\n", msg->header.stamp.toSec(), initTime.toSec());
}
else{
         ori_estimation = msg->ori;
printf("orientation message received, %g\n\n\n\n\n", ori_estimation);

}

}
*/

void foot_sequenceCallback(const interactive_walk_planner_new::foot_sequence::ConstPtr& msg)
{

  int max_no = 0;
  int slope_phase = 0;
  float highest_pos = 0;

  if (msg->header.stamp.toSec() < initTime.toSec()) {
    printf("\n\n\n\nbogus message, %g %g\n\n\n\n\n", msg->header.stamp.toSec(), initTime.toSec());
  }
  else{
          printf( "foot sequence received: \n");
         for (int i=0; i<MAX_FOOT_NO; i++){
         foot_sequence[i][0] = msg->x[i];
         foot_sequence[i][1] = msg->y[i];
         foot_sequence[i][2] = msg->theta[i];
         foot_sequence[i][3] = msg->s[i];
         foot_sequence[i][4] = msg->height[i];

         float pitch_angle = 0.0;

         if (task_mode == 0){
                 if (i>0){

                        if (msg->normal_x[i] < -0.15 && slope_phase == 0)
                                slope_phase = 1;
                        else if (msg->normal_x[i] < -0.15 && msg->normal_z[i] < 0 && slope_phase == 1)
                                slope_phase = 2;
                        else if (fabsf(msg->normal_x[i]) < 0.1 && msg->normal_z[i] > 0 && slope_phase == 2)
                                slope_phase = 0;

                        if(slope_phase == 1){
                                pitch_angle = -15.0*3.1415/180.;
                                foot_sequence[i][5] = 1;
                                foot_sequence[i][6] = 1;
                                foot_sequence[i][7] = 1;
                        }
                        else if(slope_phase == 2){
                                pitch_angle = 15.0*3.1415/180.;
                                foot_sequence[i][5] = -1;
                                foot_sequence[i][6] = -1;
                                foot_sequence[i][7] = -1;
                        }
                        else{
                                pitch_angle = 0.0;
                                foot_sequence[i][5] = 0;
                                foot_sequence[i][6] = 0;
                                foot_sequence[i][7] = 1;
                        }        
                 }                        
                 printf( "The step %d pitch value is %g \n", i, pitch_angle);
         }
         else{
                        pitch_angle = 0.0;
                        foot_sequence[i][5] = 0;
                        foot_sequence[i][6] = 0;
                        foot_sequence[i][7] = 1;
         }
                 
        
         if (i>0 && foot_sequence[i][0]==0 && foot_sequence[i][1]==0 && foot_sequence[i][2]==0){
                // hack to remove the redondant marker from previous planning
                if (longest_marker_no < i)
                        longest_marker_no = i;
                else{
                        for (int j=i; j<longest_marker_no; j++){
                         std::stringstream ss;
                         ss << j+1;
                         std::string s = ss.str();
                         geometry_msgs::Pose position;
                         position.position.x = 100;
                         position.position.y = 100;
                         position.position.z = 0;
                         position.orientation.x = 0;
                         position.orientation.y = 0;
                         position.orientation.z = 0;
                         position.orientation.w = 0;
                         makeMovingMarker(position, s, false, true);
                        }
                }
                max_no = i-1;
                break;        
         }        

         std::stringstream ss;
         ss << i+1;
         std::string s = ss.str();
         geometry_msgs::Pose position;
         position.position.x = foot_sequence[i][0];
         position.position.y = foot_sequence[i][1];
         position.position.z = foot_sequence[i][4];
         printf( "The step %d position is %g %g %g %g \n", i, foot_sequence[i][0], foot_sequence[i][1], foot_sequence[i][4], foot_sequence[i][2]);
              printf( "The step %d normal is %g %g %g \n", i, msg->normal_x[i], msg->normal_y[i], msg->normal_z[i]);
                        
         position.orientation.w = cosf(pitch_angle/2.)*cosf(foot_sequence[i][2]/2.);//q.w();        
         position.orientation.x = -sinf(pitch_angle/2.)*sinf(foot_sequence[i][2]/2.);//q.x();
         position.orientation.y = sinf(pitch_angle/2.)*cosf(foot_sequence[i][2]/2.);//q.y();
         position.orientation.z = cosf(pitch_angle/2.)*sinf(foot_sequence[i][2]/2.);;//q.z();
         if (msg->s[i] > 0)
                 makeMovingMarker(position, s, false, true);
         else
                makeMovingMarker(position, s, false, false);

         }
        
          server->applyChanges();        
         foot_sequence_update = true;        
  }
}

// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "foot_interface", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  server.reset( new interactive_markers::InteractiveMarkerServer("foot_interface","",false) );

  ros::Duration(0.1).sleep();
  initTime = ros::Time::now();


  menu_handler.insert("Clean the Path and Refresh the point cloud", &processFeedback );


  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Foot step planning part" );
  menu_handler.insert( sub_menu_handle, "Publish Goal Location and A* plan", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Publish Goal Location and Align with block", &processFeedback );
  
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle1 = menu_handler.insert( "Step for Cinder block" );
  menu_handler.insert( sub_menu_handle1, "Plan walk up cinder block", &processFeedback );
  menu_handler.insert( sub_menu_handle1, "Plan walk down cinder block", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle2 = menu_handler.insert( "Small foot adjustment" );
  menu_handler.insert( sub_menu_handle2, "small forward", &processFeedback );
  menu_handler.insert( sub_menu_handle2, "small left side walk", &processFeedback );
  menu_handler.insert( sub_menu_handle2, "small right side walk", &processFeedback );
  menu_handler.insert( "Publish the Foot Sequence to ATLAS", &processFeedback );
  
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle3 = menu_handler.insert( "Walking Mode" );
  menu_handler.insert( sub_menu_handle3, "ramp", &processFeedback );
  menu_handler.insert( sub_menu_handle3, "zig_zag", &processFeedback );
  menu_handler.insert( sub_menu_handle3, "walk up cinder block", &processFeedback );
  menu_handler.insert( sub_menu_handle3, "walk down cinder block", &processFeedback );
 
  makeMovingMarker_Goal();
  tf::Vector3 p;
  p = tf::Vector3(0, -1, 0);
  makeMenuMarker(p);
  

      geometry_msgs::Pose position;
      position.position.x = 0+0.045;
      position.position.y = 0;
      position.position.z = 0;
      position.orientation.x = 0;
      position.orientation.y = 0;
      position.orientation.z = 0;
      position.orientation.w = 1;

    std::stringstream ss;
    ss << 1;
    std::string s = ss.str();
  makeMovingMarker(position, s, false, false);
  server->applyChanges();

  goal_location_pub_ = n.advertise<interactive_walk_planner_new::goal_pose>("/interactive_walk_planner_new/goal_pose", 100, true );
  task_mode_pub_ = n.advertise<interactive_walk_planner_new::task_mode>("/interactive_walk_planner_new/task_mode", 100, true );
  replan_message_pub_ = n.advertise<interactive_walk_planner_new::replan>("/interactive_walk_planner_new/replan_message", 100, true );
  ros::Subscriber sub_foot_sequence = n.subscribe<interactive_walk_planner_new::foot_sequence>( "/atlas/planned_foot_sequence", 100, &foot_sequenceCallback );

  //ros::Subscriber sub_ori_estimation = n.subscribe<interactive_walk_planner_new::orientation>( "/orientation_estimation/orientation", 100, &ori_estimationCallback );

  pub_wrecs_step = n.advertise<atlas_ros_msgs::AtlasStepData>("/ocu/add_step_relative_left_foot", 100);


  ros::spin();
  server.reset();
}
