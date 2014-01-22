#include <iostream>
#include <time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/transformation_estimation_svd.h>

//using namespace pcl;

#include "footstep_visualizer.h"
extern "C" {
    #include "demo.h"
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ori_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
Eigen::Matrix4f Ti2 = Eigen::Matrix4f::Identity (), targetToSource2;
Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
boost::mutex mtx_;

/*
 * Visualization globals
 */

// Flag set by worker to tell visualizer to refresh data
volatile bool viewerNeedsRefresh_;
volatile bool cloudNeedsRefresh_;
// Point cloud to show after refresh
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_refresh_ptr_ (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointXYZ goal_point_;
// Footsteps to show after refresh
footsteps::FootstepVector footsteps_;
// Mutex to protect access to refresh globals
boost::mutex refreshMtx_;
// Output counter
int numOutputs_ = 0;

// Planar normals
float planar_normals[4] = {0.0f, -1.0f, 0.0f, 0.0f};

/*
 * Astar globals
 */
volatile bool planNeedsRefresh_;
volatile bool stepNeedsRefresh_;
float terrain_value[TERRAIN_N_X][TERRAIN_N_Y];
float cost_map[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_index[TERRAIN_N_X][TERRAIN_N_Y];
int terrain_count[TERRAIN_N_X][TERRAIN_N_Y];
float Ob_index[TERRAIN_N_X+50][TERRAIN_N_Y+50];
//============================ sub functions ====================================//
void Get_Data(){

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_sc (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// =========== Fill the point cloud data =====================//
	  cloud_sc->width    = TERRAIN_N_X*TERRAIN_N_Y;
	  cloud_sc->height   = 1;
	  cloud_sc->is_dense = false;
	  cloud_sc->points.resize (cloud_sc->width * cloud_sc->height);
        // ================= Generate random obstacles ================//
	srand (4000);
	int i, j; 
  	int ix, ixc, ixl, ir2;
  	int iy, iyc, iyl;
	for (i=0;i<TERRAIN_N_X;i++)
		for (j=0;j<TERRAIN_N_Y;j++)
			Ob_index[i][j]=0;	
	  
	// random rectangles: this is in pixel units
	for ( i = 0; i < 20; i++ ){
	  ixc = rand() % t1->resolution[XX];
	  iyc = rand() % t1->resolution[YY];
	  ixl = rand() % 10; // 25 is good // 30 requires errors
	  iyl = rand() % 10;
	  for ( ix = ixc;  ix <= ixc + ixl; ix++ )
	  	for ( iy = iyc; iy <= iyc + iyl; iy++ )
			Ob_index[ix][iy] = -0.2;
	}

	// random circles: this is in pixel units
	for ( i = 0; i < 20; i++ ){
		ixc = rand() % t1->resolution[XX];
		iyc = rand() % t1->resolution[YY];
		ir2 = rand() % 10;
		ixl = iyl = (int) sqrtf( (float) ir2 );
		for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ ){
		for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ ){
		    if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
		      Ob_index[ix][iy] = -0.2;
		}
		}
	}

	// random bars: this is in pixel units
	for ( i = 0; i < 15; i++ )
	{
	  ixc = rand() % t1->resolution[XX];
	  iyc = rand() % t1->resolution[YY];
	  ixl = rand() % 20; 
	  iyl = 2;
	  for ( ix = ixc;  ix <= ixc + ixl; ix++ )
	  	for ( iy = iyc; iy <= iyc + iyl; iy++ )
			Ob_index[ix][iy] = -0.2;
	}

	// free start
	terrain_indices( t1, t1->current[XX], t1->current[YY], &ixc, &iyc, NULL );
	ixl = iyl = 5;
	for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ )
	for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ )
	{
	if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
	  Ob_index[ix][iy] = 0;
	}

	// free goal
	terrain_indices( t1, t1->goal[XX], t1->goal[YY], &ixc, &iyc, NULL );
	ixl = iyl = 8;
	for ( ix = ixc - ixl;  ix <= ixc + ixl; ix++ )
	for ( iy = iyc - iyl; iy <= iyc + iyl; iy++ )
	{
	if ( (ix - ixc)*(ix - ixc) + (iy - iyc)*(iy - iyc) <= ixl*ixl )
	  Ob_index[ix][iy] = 0;
	}

	for (i = 0; i < TERRAIN_N_X; i++){
	for (j = 0; j < TERRAIN_N_Y; j++){
		cloud_sc->points[TERRAIN_N_Y*i+j].x = -2 + 0.04*(float)(i);
		cloud_sc->points[TERRAIN_N_Y*i+j].z = -1 + 0.04*(float)(j);
		cloud_sc->points[TERRAIN_N_Y*i+j].y = Ob_index[i][j];
		if (cloud_sc->points[TERRAIN_N_Y*i+j].y<0){
			cloud_sc->points[TERRAIN_N_Y*i+j].r = 165;
			cloud_sc->points[TERRAIN_N_Y*i+j].g = 42;
			cloud_sc->points[TERRAIN_N_Y*i+j].b = 42;
		}
		else{
			cloud_sc->points[TERRAIN_N_Y*i+j].r = 180;
			cloud_sc->points[TERRAIN_N_Y*i+j].g = 180;
			cloud_sc->points[TERRAIN_N_Y*i+j].b = 180;
		}
	}
	}

	boost::mutex::scoped_lock lock(refreshMtx_);
	pcl::copyPointCloud (*cloud_sc, *cloud_refresh_ptr_);		
	viewerNeedsRefresh_ = true;
	planNeedsRefresh_ = true;
	cloudNeedsRefresh_ = true;
	lock.unlock();
}

void terrain_map_pointcloud(){

	int i, j, ix, iy, LL, count, loc_i, loc_j;
	float sum;
	float temp[TERRAIN_N_X][TERRAIN_N_Y];
    	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
    	boost::mutex::scoped_lock lock (refreshMtx_);
    	pcl::PassThrough<pcl::PointXYZRGBA> pass_out;
    	pass_out.setInputCloud (cloud_refresh_ptr_);
    	pass_out.setFilterFieldName ("z");
    	pass_out.setFilterLimits (-1, 3);
    	pass_out.filter (*cloud_filtered_1);
    	pass_out.setInputCloud (cloud_filtered_1);
    	pass_out.setFilterFieldName ("x");
    	pass_out.setFilterLimits (-2, 2);
    	pass_out.filter (*cloud_filtered_2);
    	lock.unlock();
		
    	LL = cloud_filtered_2->height*cloud_filtered_2->width;

    	for (i=0;i<LL;i++){
	if (cloud_filtered_2->points[i].x != 0){
		terrain_indices( t1, cloud_filtered_2->points[i].x, cloud_filtered_2->points[i].z, &ix, &iy, NULL );
		terrain_value[ix][iy] = terrain_value[ix][iy] - cloud_filtered_2->points[i].y;
		terrain_count[ix][iy] = terrain_count[ix][iy] + 1;
		terrain_index[ix][iy] = i;

      	}
    	}

    	for (i=0 ; i<TERRAIN_N_X ; i++){
      		for (j=0 ; j<TERRAIN_N_Y ; j++){
			if (terrain_count[i][j]!=0)
	  			terrain_value[i][j] = terrain_value[i][j]/terrain_count[i][j];
			temp[i][j] = terrain_value[i][j];
      	}
    	}

	for (i=0 ; i<TERRAIN_N_X ; i++){
	for (j=0 ; j<TERRAIN_N_Y ; j++){
	if (temp[i][j]==0){
	  count = 0;
	  sum = 0;
	  for (loc_i=i-2; loc_i<i+2; loc_i++){
	    for (loc_j=j-2; loc_j<j+2; loc_j++){
	      if (loc_i>=0&&loc_j>=0&&loc_i<TERRAIN_N_X&&loc_j<TERRAIN_N_X){
		if (temp[loc_i][loc_j]!=0){
		  sum = sum + temp[loc_i][loc_j];
		  count = count + 1;
		}
	      }
	    }
	  }
	  if (count>0)
	    terrain_value[i][j] = sum/count;
	}
	}
	}
}

void terrain_map_random(){
	int i, j, count, loc_i, loc_j;
	float sum;
	float temp[TERRAIN_N_X][TERRAIN_N_Y];

	for (i=0 ; i<TERRAIN_N_X ; i++)
	for (j=0 ; j<TERRAIN_N_Y ; j++)
		temp[i][j] = -Ob_index[i][j];

	for (i=0 ; i<TERRAIN_N_X ; i++){
	for (j=0 ; j<TERRAIN_N_Y ; j++){
	terrain_value[i][j] = temp[i][j];
	if (temp[i][j]==0){
	  count = 0;
	  sum = 0;
	  for (loc_i=i-2; loc_i<i+2; loc_i++){
	    for (loc_j=j-2; loc_j<j+2; loc_j++){
	      if (loc_i>=0&&loc_j>=0&&loc_i<TERRAIN_N_X&&loc_j<TERRAIN_N_X){
		if (temp[loc_i][loc_j]!=0){
		  sum = sum + temp[loc_i][loc_j];
		  count = count + 1;
		}
	      }
	    }
	  }
	  if (count>0)
	    terrain_value[i][j] = sum/count;
	}
	}
	}
}

void pathplanFunc()
{
	int i, j, input1;
	clock_t start, end;
	int ix, iy, LL;
	float kx, ky, sum;
	float NextStep_pos[2], current_point[2];
	FILE * pFile;

	NextStep_pos[0] = 0.0f;
	NextStep_pos[1] = 0.0f;
        current_point[0] = t1->current[XX];
	current_point[1] = t1->current[YY];
	
	while(1){
	if (planNeedsRefresh_){

		planNeedsRefresh_ = false;			
		t1->current[XX] = current_point[0];
		t1->current[YY] = current_point[1];	

		if (t1->current[SIDE] == LEFT)
			t1->current[SIDE] = RIGHT;
		else
			t1->current[SIDE] = LEFT;	

		start = clock();

		for (i=0;i<TERRAIN_N_X;i++){
			for (j=0;j<TERRAIN_N_Y;j++){
				terrain_value[i][j] = 0;
				terrain_count[i][j] = 0;
				terrain_index[i][j] = 0;
		}
		}
		// Create terrain map based on point cloud data
		//terrain_map_pointcloud();
		// Create terrain map based on random map
		terrain_map_random();

		// Find the path based on Astar
		generate_true_cost_map( t1, terrain_value);

		a1 = create_astar( t1 );

		plan();

		printf_final_path( p1 );

		// Generate footsteps
		footsteps::FootstepVector steps;

		if (p1->previous !=NULL){
			NextStep_pos[0] = p1->previous->state[XX];
			NextStep_pos[1] = p1->previous->state[YY];
		}	

	  	 // get indices of target pcl points
		  while (p1 != NULL)
		  {
		      // footstep parameters

		      pcl::PointXYZ target_pt = pcl::PointXYZ(p1->state[XX], 0.0f, p1->state[YY]); 

		      float rotation = -(3.1416f/2.0f - p1->state[ANGLE]);
		      footsteps::Chirality::Kind chirality = (footsteps::Chirality::Kind)p1->state[SIDE];

		      // create new point to be footstep target
		      pcl::PointNormal pt_nrm;
		      memcpy(pt_nrm.data, target_pt.data, 3 * sizeof(float)); // copy xyz

		      // Dummy normal vector
		      memcpy(pt_nrm.data_n, planar_normals, 3 * sizeof(float));
		      footsteps::Footstep footstep (pt_nrm, rotation, chirality);
		      steps.push_back(footstep);
		      p1 = p1->previous;
		  }
		  std::cout << "next step location" << NextStep_pos[0] << " " << NextStep_pos[1] <<std::endl;	
		  end = clock();
		  std::cout << "One step planning time is" << (end-start)/(1000) << " ms" << std::endl;
		  // Update refresh global variables
		  {
		    goal_point_ = pcl::PointXYZ(0.0f, 0.0f, 3.0f);
		    boost::mutex::scoped_lock lock(refreshMtx_);
		    footsteps_ = steps;
		    viewerNeedsRefresh_ = true;
		    stepNeedsRefresh_ = true;	
		    lock.unlock();
		  }
		  	
  }	
  }
}

void visualizerFunc()
{
  boost::shared_ptr<footsteps::FootstepVisualizer> viewer;
	// Create visualizer

  viewer = boost::shared_ptr<footsteps::FootstepVisualizer>(new footsteps::FootstepVisualizer ("Viewer"));

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters();

  viewer->setCameraPosition(-4.28936,-4.30569,-4.22572, -0.0236325,-0.856731,0.515222);

	while(!viewer->wasStopped())
	{
    // Refresh data if necessary
    if (viewerNeedsRefresh_)
    {
      viewerNeedsRefresh_ = false;
      boost::mutex::scoped_lock lock (refreshMtx_);
      if (cloudNeedsRefresh_ == true){	
	      cloudNeedsRefresh_ == false;
	      // --------------------------------------------
	      // -----Remove old point cloud and add point cloud-----
	      // --------------------------------------------
	      const std::string pc_id = "cloud";
	      if (!viewer->updatePointCloud(cloud_refresh_ptr_, pc_id))
	      {
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_refresh_ptr_);
		viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_refresh_ptr_, rgb, pc_id);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pc_id);
	      }
      }
      if (stepNeedsRefresh_ == true){
	      stepNeedsRefresh_ == false;
	      // -------------------------------------------------------
	      // ------Remove old footsteps and add new footsteps-------
	      // -------------------------------------------------------
	      const std::string fs_id = "footsteps";
	      viewer->removeFootsteps(fs_id);
	      viewer->addFootsteps(footsteps_, fs_id);

	      //if (!viewer->updateSphere<pcl::PointXYZ>(goal_point_, 0.2f, 1, 0, 0, "sphere"))
		  //viewer->addSphere<pcl::PointXYZ>(goal_point_, 0.2f, 1, 0, 0,"sphere");
      }
      lock.unlock();
    }

    viewer->spinOnce(1);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}
}

int
main (int argc, char** argv)
{
	int i, j, input1;
	// =============== Creat an empty Terrain size 4mx4m=================//
	t1 = create_terrain();

    	t1->min[XX] = -2.0;
    	t1->min[YY] = -1.0;
    	t1->max[XX] = 2.0;
    	t1->max[YY] = 3.0;

  	// current and goal states
  	t1->current[XX] = -1.5;
  	t1->current[YY] = -0.5;
  	t1->current[ANGLE] = (float) (M_PI/2);
  	t1->current[SIDE] = LEFT;

  	t1->goal[XX] = 1.5;
  	t1->goal[YY] = 2.5;
  	t1->goal[ANGLE] = t1->current[ANGLE];
  	t1->current[SIDE] = LEFT; 

	// See how far away perception horizon is.
	t1->perception_radius = 2*TERRAIN_N_X;

	Get_Data();


	// Threading
	boost::thread pathplanThread(pathplanFunc);
	boost::thread visualizerThread(visualizerFunc);

	boost::thread_group group;
	group.add_thread(&visualizerThread);
	group.add_thread(&pathplanThread);

	group.join_all();

        std::cout << "Select data number: ";
        std::cin >> input1;
        //interface->stop();
        return (0);
}
