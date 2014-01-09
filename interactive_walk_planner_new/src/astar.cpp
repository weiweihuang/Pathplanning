/*******************************************************************/
/*
demo.c: demo astar
*/
/*******************************************************************/

#include "astar.h"

/******* GLOBALS *****************************************************/

// Seems OpenGL requires us to have globals
TERRAIN *t1 = NULL; // terrain map
ASTAR *a1 = NULL; // A* search tree
PATH_NODE *p1 = NULL; // actual path taken
int path_done = 0; // flag: is goal reached by robot?
int wa = -1; // Window for ASTAR stuff

// debugging flags
int debug_a = 0;
int debug_plan = 0;
int debug_free = 0;

// for debugging
int astar_node_count = 0; // needs to be global since free list shared across

//Action sets
float action16_l[12][3];
float action16_r[12][3];
float continus_cost[10][10];
extern int task_mode;
extern std::string ActionPath;
float angle_heuristic = 0.0;


/******* Load Action Set ********************************************/
void load_actions(){
  int i, j;
  float f1, f2,f3;
  // Read txt file
  FILE *my_stream;
  char buf[1024];
  strcpy(buf, ActionPath.c_str());
  strcat(buf, "actionset10_ls_new.txt");
  my_stream=fopen(buf, "r");
  if (my_stream==NULL){
    printf("Cannot open the file");
    fclose(my_stream);
  }
  else{
    for(i=0; i<12; i++){
      fscanf(my_stream,"%f %f %f\n", &f1, &f2, &f3);
      action16_l[i][0]=f2;
      action16_l[i][1]=f1;
      action16_l[i][2]=(f3-90)*M_PI/180.0;
    }
    fclose(my_stream);
  }

  strcpy(buf, ActionPath.c_str());
  strcat(buf, "actionset10_rs_new.txt");
  my_stream=fopen(buf, "r");
  if (my_stream==NULL){
    printf("Cannot open the file");
    fclose(my_stream);
  }
  else{
    for(i=0; i<12; i++){
      fscanf(my_stream,"%f %f %f\n", &f1, &f2, &f3);
      action16_r[i][0]=f2;
      action16_r[i][1]=f1;
      action16_r[i][2]=(f3-90)*M_PI/180.0;
    }
    fclose(my_stream);
  }


  strcpy(buf, ActionPath.c_str());
  strcat(buf, "relationship_cost.txt");
  my_stream=fopen(buf, "r");
  if (my_stream==NULL){
    printf("Cannot open the file");
    fclose(my_stream);
  }
  else{
    for(i=0; i<10; i++){
        for (j=0; j<10; j++){
                      fscanf(my_stream,"%f", &f1);
                      continus_cost[i][j]=f1;
                fprintf( stderr, "%g ", continus_cost[i][j]);
        }
        fscanf(my_stream,"\n");
        fprintf( stderr, "\n");
    }
    fclose(my_stream);
  }

}

/******* TERRAIN ****************************************************/

TERRAIN *create_terrain()
{
  int i;
  TERRAIN *tt;

  tt = (TERRAIN *) malloc( sizeof( TERRAIN ) );
  if ( tt == NULL )
  {
    fprintf( stderr, "Can't allocate terrain.\n" );
    exit( -1 );
  }

  tt->resolution[XX] = TERRAIN_N_X;
  tt->resolution[YY] = TERRAIN_N_Y;
  tt->n_cells = tt->resolution[XX]*tt->resolution[YY];

  tt->true_cost_map = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->true_cost_map == NULL )
  {
    fprintf( stderr, "Can't allocate true terrain cost map.\n" );
    exit( -1 );
  }

  tt->perceived_cost_map = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->perceived_cost_map == NULL )
  {
    fprintf( stderr, "Can't allocate perceived terrain cost map.\n" );
    exit( -1 );
  }

  tt->known_terrain = (float *) malloc( tt->n_cells*sizeof( float ) );
  if ( tt->known_terrain == NULL )
  {
    fprintf( stderr, "Can't allocate known terrain cost map.\n" );
    exit( -1 );
  }

  for ( i = 0; i < tt->n_cells; i++ )
  {
    tt->true_cost_map[i] = 0;
    tt->perceived_cost_map[i] = 0;
    tt->known_terrain[i] = 0;
  }

  return tt;
}

/*******************************************************************/

void terrain_indices( TERRAIN *tt, float x, float y,
    int *ix, int *iy, int *index )
{
  int iix, iiy;
  if ( x == tt->max[XX] || x > tt->max[XX])
    iix = tt->resolution[XX] - 1;
  else if ( x < tt->min[XX])
    iix = 0;        
  else
    iix = (int) (tt->resolution[XX]*(x - tt->min[XX])
        /(tt->max[XX] - tt->min[XX]));

  if (iix >TERRAIN_N_X -1)
    iix = TERRAIN_N_X -1;

  if ( y == tt->max[YY] || y > tt->max[YY])
    iiy = tt->resolution[YY] - 1;
  else if ( y < tt->min[YY])
    iiy = 0;        
  else
    iiy = (int) (tt->resolution[YY]*(y - tt->min[YY])
        /(tt->max[YY] - tt->min[YY]));

  if (iiy >TERRAIN_N_Y -1)
    iiy = TERRAIN_N_Y -1;

  if ( ix != NULL )
    *ix = iix;

  if ( iy != NULL )
    *iy = iiy;


  if ( index != NULL )
    *index = iix*tt->resolution[YY] + iiy;
}

/*******************************************************************/

float get_perceived_cost_map_pixel( TERRAIN *tt, int ix, int iy )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return BIG_COST;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return BIG_COST;

  index = ix*tt->resolution[YY] + iy;

  return tt->perceived_cost_map[ index ];
}

/*******************************************************************/

float get_true_cost_map_pixel( TERRAIN *tt, int ix, int iy )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return 0;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return 0;

  index = ix*tt->resolution[YY] + iy;

  return tt->true_cost_map[ index ];
}

/*******************************************************************/

void set_perceived_cost_map_pixel( TERRAIN *tt, int ix, int iy, float value )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return;

  index = ix*tt->resolution[YY] + iy;

  tt->perceived_cost_map[ index ] = value;
  tt->known_terrain[ index ] += 1.0;
}

/*******************************************************************/

void set_true_cost_map_pixel( TERRAIN *tt, int ix, int iy, float value )
{
  int index;

  if ( ix < 0 || ix >= tt->resolution[XX] )
    return;

  if ( iy < 0 || iy >= tt->resolution[YY] )
    return;

  index = ix*tt->resolution[YY] + iy;

  tt->true_cost_map[ index ] = value;
}

/*******************************************************************/

void generate_true_cost_map( TERRAIN *tt, float terrain_value[TERRAIN_N_X][TERRAIN_N_Y] )
{
  int i, j, index;
  int ix, ixc;
  int iy, iyc;
  int ixc2, iyc2;
  int kx, ky;
  float value[TERRAIN_N_X][TERRAIN_N_Y];
  float filter_coeff = 0.6f;

  tt->inc = (tt->max[XX] - tt->min[XX])/TERRAIN_N_X;
  if ( fabsf( tt->inc - (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y ) > 1e-3 )
  {
    // Need to implement more sophisticated tt->inc
    fprintf( stderr, "X and Y distance/pixel not equal: %g %g\n",
        tt->inc, (tt->max[YY] - tt->min[YY])/TERRAIN_N_Y );
    exit( -1 );
  }

  // initialize cost map
  for ( i = 0; i < tt->resolution[XX]; i++ ){
    for ( j = 0; j < tt->resolution[YY]; j++ ){
      set_true_cost_map_pixel( tt, i, j, terrain_value[i][j] );
      index = i*tt->resolution[YY] + j;
      tt->perceived_cost_map[ index ] = 0;
      tt->known_terrain[ index ] = 0;
    }
  }                 
}

/******* A* SEARCH ****************************************************/
/*******************************************************************/
// used in generate_child(), heuristic()
// should scale by dimension (x,y vs. angle)
// ignore side


float angle_diff (float a1, float a2){

        float angle1 = 180.0 * a1/M_PI;
        float angle2 = 180.0 * a2/M_PI;

        float angle = angle1 - angle2;

         while (angle < 0){
                angle = angle + 360;
         }
        angle = fmod((angle + 180.0),360.0) -180.0;
        return angle*M_PI/180.0;
 
}

float xya_distance( ASTAR *aa, float *s1, float *s2 )
{
  int i, j;
  float diff;
  float score = 0;

  // X. Y distance
  for ( i = XX; i <= YY; i++ )
  {
    diff = s1[i] - s2[i];
    score += diff*diff;
  }

  // ANGLE
  i = ANGLE;
  diff = angle_diff (s1[i], s2[i]);

  score += angle_heuristic*diff*diff;

  return sqrtf( score );
}

/*******************************************************************/
// should scale by dimension (x,y vs. angle)

float state_distance( ASTAR *aa, float *s1, float *s2 )
{
  int i;
  float diff1, diff2;
  float score = 0;

  score = xya_distance( aa, s1, s2 );

  // angle distance
  i = ANGLE;
  diff1 = s1[i] - s2[i];

  // side distance
  i = SIDE;
  diff2 = s1[i] - s2[i];

  // side is not a meaningful Euclidean distance anyway
  return score + 0.0*diff1*diff1 + diff2*diff2;
}

/*******************************************************************/

ASTAR_NODE* create_astar_node( ASTAR *aa )
{
  ASTAR_NODE *an;

  if ( aa->free_list != NULL )
  {
    an = aa->free_list;
    aa->free_list = an->next;
  }
  else
  {
    an = (ASTAR_NODE *) malloc( sizeof( ASTAR_NODE ) );
    an->id = astar_node_count++;
  }

  if ( an == NULL )
  {
    fprintf( stderr, "Can't allocate astar node.\n" );
    exit( -1 );
  }
  an->depth = -1;
  an->state[XX] = -1e10;
  an->state[YY] = -1e10;
  an->state[ANGLE] = -1e10;
  an->state[SIDE] = -1;
  an->state[PITCH] = -1e10;
  an->com[XX] = -1e10;
  an->com[YY] = -1e10;
  an->com[ZZ] = -1e10;
  an->com[ANGLE] = -1e10;
  an->com[SIDE] = -1;
  an->com[PITCH] = -1e10;
  an->terrain_index = -1;
  an->location_index = -1;
  an->parent = NULL;
  an->children = NULL;
  an->sibling = NULL;
  an->next = NULL;
  an->location_next = NULL;
  an->cost_from_start = -1;
  an->one_step_cost = -1;
  an->terrain_cost = -1;
  an->cost_to_go = -1;
  an->priority = -1;

  return an;
}

/*******************************************************************/

void free_astar_node( ASTAR *aa, ASTAR_NODE *an )
{
  if ( an == NULL )
    return;

  if ( an->parent != NULL || an->children != NULL || an->sibling != NULL
      || an->next != NULL || an->location_next != NULL )
  {
    fprintf( stderr, "Can't free ASTAR_NODE that is in use.\n" );
    exit( -1 );
  }

  if ( debug_free )
    printf( "fan: %d\n", an->id );

  an->depth = 0;

  an->next = aa->free_list;
  aa->free_list = an;

  if ( debug_free )
    printf( "fan 2: %d\n", an->id );
}

/*******************************************************************/
// traversing children, and sibling links should do it.

void free_all_astar_nodes( ASTAR *aa, ASTAR_NODE *an )
{
  ASTAR_NODE *an_next;

  if ( an == NULL )
    return;

  if ( debug_free )
    printf( "faan: %d\n", an->id );

  for ( ; ; )
  {
    if ( an == NULL )
      return;

    an_next = an->next;

    an->parent = NULL;
    an->sibling = NULL;
    an->children = NULL;
    an->next = NULL;
    an->location_next = NULL;

    if ( debug_free )
      printf( "faan 2: %d\n", an->id );

    free_astar_node( aa, an );

    if ( debug_free )
      printf( "faan 3\n" );

    an = an_next;
  }

  if ( debug_free )
    printf( "faan 4\n" );
}

/*******************************************************************/

void free_astar( ASTAR *aa )
{
  if ( aa == NULL )
  {
    fprintf( stderr, "NULL aa in free_astar()\n" );
    exit( -1 );
  }

  aa->active = 0;

  if ( debug_free )
    printf( "free_astar 100\n" );

  free_all_astar_nodes( aa, aa->priority_q );

  if ( debug_free )
    printf( "free_astar 110\n" );

  free_all_astar_nodes( aa, aa->done_list );

  if ( debug_free )
    printf( "free_astar 120\n" );

  free_all_astar_nodes( aa, aa->at_goal );

  if ( debug_free )
    printf( "free_astar 130\n" );

  aa->priority_q = NULL;
  aa->done_list = NULL;
  aa->at_goal = NULL;
  aa->root_node = NULL;

  if ( debug_free )
    printf( "free_astar 200\n" );
}

/*******************************************************************/

float heuristic( ASTAR *aa, ASTAR_NODE *an )
{

  if ( an->state[XX] < aa->tt->min[XX] )
    return BIG_VALUE;
  if ( an->state[YY] < aa->tt->min[YY] )
    return BIG_VALUE;
  if ( an->state[XX] > aa->tt->max[XX] )
    return BIG_VALUE;
  if ( an->state[YY] > aa->tt->max[YY] )
    return BIG_VALUE;

  if ( an->com[XX] < aa->tt->min[XX] )
    return BIG_VALUE;
  if ( an->com[YY] < aa->tt->min[YY] )
    return BIG_VALUE;
  if ( an->com[XX] > aa->tt->max[XX] )
    return BIG_VALUE;
  if ( an->com[YY] > aa->tt->max[YY] )
    return BIG_VALUE;

  float heuristic = xya_distance( aa, an->com, aa->tt->goal );

  // use simple distance to goal as heuristic function
  return heuristic;
}


/*******************************************************************/
float astar_step_cost( ASTAR *aa, ASTAR_NODE *an_child, ASTAR_NODE *an )
{

  // new version
  int index, action_index, pre_index;
  float step_height, current_height, perent_height;
  float step_cost_index[10] = {1.0, 0.6, 0.7, 0.8, 1.5, 2.0, 2.0, 1.2, 1.6, 1.8};

  int danger_step = 0;


  step_height = fabsf(an_child->state[ZZ]-an->state[ZZ]);

  if (an_child->action_index > 9)
        action_index = 3;
  else
        action_index = an_child->action_index;        

  if (an->action_index > 9)
        pre_index = 3;
  else
        pre_index = an->action_index;        


  float step_cost_current = step_cost_index[action_index];

  float step_cost_continus = 0;
  if (step_height >0.1 && action_index==0)
        danger_step = 1;

  if (step_height >0.1 && angle_diff(an_child->state[ANGLE],0)>20*M_PI/180.0)
        danger_step = 1;

  if (step_height >0.1 && action_index>6)
        danger_step = 1;

  if (action_index<0 || action_index>9 || action_index < 0 || action_index > 9)
        danger_step = 1;
  else        
        step_cost_continus = 10*continus_cost[action_index][pre_index ];

  if (step_cost_continus > 2.0)
         danger_step = 1;

  float step_cost = (step_cost_current + step_cost_continus);

  //printf( "step cost value is : %g with index %d %d \n", step_cost, an_child->action_index, an->action_index);

  if (danger_step == 1)
        return BIG_COST;
  else
          return step_cost;

}

int astar_swing_cost(float x1, float y1, float x2, float y2, float max_h, ASTAR *aa)
{
  int i, j, index, count=0, print =0;
  float swing_cost=0, a, kk, line;
  float min_x, min_y, max_x, max_y;
  float x, y, terrain_height, start_height, end_height, diff_height;

  //if (x2>1.31 && x2<1.41)
        //print =1;        
  

  if (x1>x2){
    min_x = x2;
    max_x = x1;
  }
  else{
    min_x = x1;
    max_x = x2;
  }
  if (y1>y2){
    min_y = y2;
    max_y = y1;
  }
  else{
    min_y = y1;
    max_y = y2;
  }

  if (fabsf(y2-y1)<fabsf(x2-x1)){
    kk = (y2-y1)/(x2-x1);
    a = kk*x1-y1;
  }
  else {
    kk = (x2-x1)/(y2-y1);
    a = kk*y1-x1;
  }


  for (i=(int)(min_x/DISTANCE)-2;i<(int)(max_x/DISTANCE)+2;i++){
    for (j=(int)(min_y/DISTANCE)-2;j<(int)(max_y/DISTANCE)+2;j++){
      if (fabsf(y2-y1)<fabsf(x2-x1))
        line = DISTANCE*j + a - kk*DISTANCE*i;
      else
        line = DISTANCE*i + a - kk*DISTANCE*j;
      if (line < 0.1 && line > -0.1){
        x = DISTANCE*i;
        y = DISTANCE*j;        
        if ( x < aa->tt->min[XX]
            || x > aa->tt->max[XX]
            || y < aa->tt->min[YY]
            || y > aa->tt->max[YY] )
          terrain_height = 0;
        else{
          terrain_indices( aa->tt, x, y, NULL, NULL, &index);
          terrain_height = aa->tt->perceived_cost_map[index];
        }
        if (terrain_height>0.075+max_h){
          count = count + 1;
        }
      }
    }
  }

    return count;
}

//Eigen::Matrix<double,3,1> paramp;
//Eigen::MatrixXd Xp(91,3); // This is for plane fitting
//Eigen::MatrixXd Z(91,1);
/*******************************************************************/
void Get_terrain_cost( ASTAR *aa, ASTAR_NODE *an, float x, float y, float angle_factor, float terrain_feedback[2], float *foot_pitch)
{        
  int ix, iy, index, plan_count=0, danger_count=0;
  float px_c, py_c, px, py, diff_height=0, s, c, angle;
  int foot_x_size = 13;
  int foot_y_size = 7;        

  float step_height = -1.0;
  float return_value = 0;

  if ( x < aa->tt->min[XX]
      || x > aa->tt->max[XX]
      || y < aa->tt->min[YY]
      || y > aa->tt->max[YY] ){

        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = 0;
            return;
        }

  angle = an->state[ANGLE] + angle_factor;
  s = sinf(angle);
  c = cosf(angle);

  for(ix=3;ix<foot_x_size-3;ix++)
  {
    for(iy=2;iy<foot_y_size-2;iy++)
    {
      px_c = DISTANCE*ix-(foot_x_size-1)/2.0*DISTANCE;
      py_c = DISTANCE*iy-(foot_y_size-1)/2.0*DISTANCE;
      px = px_c*c-py_c*s + x;
      py = px_c*s+py_c*c + y;
      terrain_indices( aa->tt, px, py, NULL, NULL, &index);         
      if (aa->tt->perceived_cost_map[index]!=0){
         if (aa->tt->perceived_cost_map[index] > step_height)
                step_height = aa->tt->perceived_cost_map[index];
        
      }        
    }
  }

  if (step_height ==-1)
        step_height = 0;

  //publish step down when goal is high
  if (step_height - an->state[ZZ]<-0.10 && task_mode == 2){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
            return;
  }

  //publish step up when goal is low
  if (step_height - an->state[ZZ]>0.10 && task_mode == 3){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
            return;
  }


  //maximun height change
  diff_height = fabsf(step_height - an->state[ZZ]);
  if (diff_height>0.20){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
            return;
  }


  // supporting region cost
  int bellow_support_count = 0;
  int above_support_count = 0;
  int total_count = 0;

  for(ix=0;ix<foot_x_size;ix++)
  {
    for(iy=0;iy<foot_y_size;iy++)
    {
      px_c = DISTANCE*ix-(foot_x_size-1)/2.0*DISTANCE;
      py_c = DISTANCE*iy-(foot_y_size-1)/2.0*DISTANCE;
      px = px_c*c-py_c*s + x;
      py = px_c*s+py_c*c + y;
      terrain_indices( aa->tt, px, py, NULL, NULL, &index);
      if (aa->tt->perceived_cost_map[index]!=0){        
         if (aa->tt->perceived_cost_map[index] > step_height + 0.05)
                above_support_count++;                                        
         if (aa->tt->perceived_cost_map[index] < step_height - 0.05)
                bellow_support_count++;        

         total_count++;        
      }        
    }
  }

  if ((float)(above_support_count)/(float)(total_count) > 0.1 || (float)(bellow_support_count)/(float)(total_count) > 0.25){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
            return;
  }                        

/*
// Get the max height
for(ix=0;ix<foot_x_size;ix++)
{
for(iy=0;iy<foot_y_size;iy++)
{
px_c = DISTANCE*ix-(foot_x_size-1)/2.0*DISTANCE;
py_c = DISTANCE*iy-(foot_y_size-1)/2.0*DISTANCE;
px = px_c*c-py_c*s + x;
py = px_c*s+py_c*c + y;
terrain_indices( aa->tt, px, py, NULL, NULL, &index);         
if (aa->tt->perceived_cost_map[index]!=0){
         Xp(plan_count,0) = px_c; //x
         Xp(plan_count,1) = py_c; //y
         Xp(plan_count,2) = 1;
         Z(plan_count) = aa->tt->perceived_cost_map[index];
         if (ix>1 && ix < foot_x_size-2 && iy>1 && iy < foot_y_size-2){
                 if (aa->tt->perceived_cost_map[index] > step_height)
                        step_height = aa->tt->perceived_cost_map[index];
         }        
         plan_count++;        
}        
}
}

Eigen::MatrixXd New_Xp(plan_count,3); // This is for plane fitting
Eigen::MatrixXd New_Z(plan_count,1);

for (int i=0; i<plan_count; i++){
         New_Xp(i,0) = Xp(i,0); //x
         New_Xp(i,1) = Xp(i,1); //y
         New_Xp(i,2) = 1;
         New_Z(i) = Z(i);
}



// get the foot pitch value
if (plan_count> 5){
         Eigen::JacobiSVD<Eigen::MatrixXd> svdp(New_Xp, Eigen::ComputeThinU | Eigen::ComputeThinV);
         paramp = svdp.solve(New_Z);
}
else{
        paramp(0,0) = 0;
        paramp(1,0) = 0;
        paramp(2,0) = 0;
}
*foot_pitch = paramp(1,0);        


// for surrounding cost
for(ix=-2;ix<foot_x_size+2;ix++){
for(iy=-1;iy<foot_y_size+1;iy++)
{
if(ix<0 || ix> foot_x_size || iy<0 || iy>foot_y_size){        
         px_c = DISTANCE*ix-(foot_x_size-1)/2.0*DISTANCE;
py_c = DISTANCE*iy-(foot_y_size-1)/2.0*DISTANCE;
         px = px_c*c-py_c*s + x;
         py = px_c*s+py_c*c + y;
         terrain_indices( aa->tt, px, py, NULL, NULL, &index);
         if (aa->tt->perceived_cost_map[index] > step_height+0.10)
                danger_count++;        
        }        
}
}
if (danger_count>1){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
        return;
}

float rough_cost;
int rough_count=0;
for(ix=0;ix<foot_x_size;ix++)
{
for(iy=0;iy<foot_y_size;iy++)
{
px_c = DISTANCE*ix-(foot_x_size-1)/2.0*DISTANCE;
py_c = DISTANCE*iy-(foot_y_size-1)/2.0*DISTANCE;
px = px_c*c-py_c*s + x;
py = px_c*s+py_c*c + y;
terrain_indices( aa->tt, px, py, NULL, NULL, &index);
float height_p = paramp(0,0)*px_c+paramp(1,0)*py_c+paramp(2,0);
if (aa->tt->perceived_cost_map[index]!=0){        
        rough_cost = rough_cost + fabsf(aa->tt->perceived_cost_map[index] - height_p);
        rough_count++;
}        
}
}
rough_cost = rough_cost/(float)(rough_count);

if (rough_cost > 100){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
        return;
}
*/

  // for swing cost
  if (task_mode == 1){
         float swing_distance = 0;
         float swing_xx, swing_yy;
         if (an->parent==NULL){        
                 terrain_indices( aa->tt, an->state[XX], an->state[YY]-HIP_WIDTH, NULL, NULL, &index);
                swing_xx = x - an->state[XX];
                swing_yy = y - (an->state[YY]-HIP_WIDTH);
                swing_distance = sqrtf(swing_xx *swing_xx + swing_yy*swing_yy);
         }
         else{
                terrain_indices( aa->tt, an->parent->state[XX], an->parent->state[YY], NULL, NULL, &index);
                swing_xx = x - an->parent->state[XX];
                swing_yy = y - an->parent->state[YY];
                swing_distance = sqrtf(swing_xx *swing_xx + swing_yy*swing_yy);
         }

         if (step_height > aa->tt->perceived_cost_map[index] + 0.2 || step_height < aa->tt->perceived_cost_map[index] - 0.2){
                terrain_feedback[0] = BIG_COST;
                terrain_feedback[1] = step_height;
                 return;
         }

         if (fabsf(step_height - aa->tt->perceived_cost_map[index]) > 0.1 && swing_distance > 0.4){
                terrain_feedback[0] = BIG_COST;
                terrain_feedback[1] = step_height;
                 return;
         }
        
         float height_level;

         if (aa->tt->perceived_cost_map[index]>step_height)
                height_level = aa->tt->perceived_cost_map[index];
         else
                height_level = step_height;

         int s_count = 0;

         if (an->parent==NULL)        
                s_count = astar_swing_cost(an->state[XX], an->state[YY]-HIP_WIDTH, x, y, height_level, aa);
         else        
                 s_count = astar_swing_cost(an->parent->state[XX], an->parent->state[YY], x, y, height_level, aa);

         if (s_count>1){
                terrain_feedback[0] = BIG_COST;
                terrain_feedback[1] = step_height;
                 return;
         }
  }
  // for over lay cost
/*
float dis_two_feet = sqrt((x-an->state[XX])*(x-an->state[XX]) + (y-an->state[YY])*(y-an->state[YY]));
if (dis_two_feet < 0.1){
        terrain_feedback[0] = BIG_COST;
        terrain_feedback[1] = step_height;
        return;
}

*/

        terrain_feedback[0] = 0;
        terrain_feedback[1] = step_height;
            return;
}


ASTAR *create_astar( TERRAIN *tt )
{
  int i;
  ASTAR *aa;

  aa = (ASTAR *) malloc( sizeof( ASTAR ) );
  if ( aa == NULL )
  {
    fprintf( stderr, "Can't allocate astar.\n" );
    exit( -1 );
  }
  aa->tt = tt;

  aa->location_map =
    (ASTAR_NODE **) malloc( tt->n_cells*sizeof(ASTAR_NODE *) );
  if ( aa->location_map == NULL )
  {
    fprintf( stderr, "Can't allocate astar location map.\n" );
    exit( -1 );
  }

  aa->active = 0;
  aa->done = 0;
  aa->priority_q = NULL;
  aa->done_list = NULL;
  aa->free_list = NULL;
  aa->at_goal = NULL;
  aa->root_node = NULL;

  for ( i = 0; i < tt->n_cells; i++ )
    aa->location_map[i] = NULL;

  aa->policy = 0;
  //aa->policy = 2;
  aa->best_value = BIG_VALUE;

  aa->expand_next_node_count = 0;

  return aa;
}

/*******************************************************************/

void init_astar( ASTAR *aa )
{
  int i;
  int index;
  ASTAR_NODE *an;
  TERRAIN *tt;
  float s, c, y_offset;

  tt = aa->tt;

  // initialize data structure
  aa->done = 0;

  // aa->free_list: This is preserved across uses.
  if ( aa->priority_q != NULL || aa->done_list != NULL || aa->at_goal != NULL
      || aa->root_node != NULL )
  {
    fprintf( stderr, "Non null pointers in init_astar\n" );
    exit( -1 );
  }

  for ( i = 0; i < tt->n_cells; i++ )
    aa->location_map[i] = NULL;

  // create first node
  an = create_astar_node( aa );

  an->depth = 0;
  an->action_index = 4;

  an->state[XX] = tt->current[XX];
  an->state[YY] = tt->current[YY];
  an->state[ZZ] = 0.0;
  an->state[ANGLE] = tt->current[ANGLE];
  an->state[SIDE] = tt->current[SIDE];
  an->state[PITCH] = 0.0;

  if ( an->state[SIDE] < 0.1 )
  {
    y_offset = -HIP_WIDTH*STEP_WIDTH_FACTOR;
  }
  else
  {
    y_offset = +HIP_WIDTH*STEP_WIDTH_FACTOR;
  }

  an->com[ANGLE] = an->state[ANGLE];
  s = sinf( an->state[ANGLE] );
  c = cosf( an->state[ANGLE] );
  an->com[XX] = an->state[XX] + s*y_offset/2;
  an->com[YY] = an->state[YY] - c*y_offset/2;

  terrain_indices( aa->tt, an->state[XX], an->state[YY], NULL, NULL, &index );
  if ( index < 0 || index >= tt->n_cells )
  {
    fprintf( stderr, "Current out of bounds %d %d\n", index, tt->n_cells );
    exit( -1 );
  }
  // currently all of these are the same, but eventually they may be different
  an->terrain_index = index;
  an->location_index = index;

  an->cost_from_start = 0.0;
  an->one_step_cost = 0.0;
  an->terrain_cost = 0.0;
  an->cost_to_go = heuristic( aa, an );
  an->priority = an->cost_from_start + INFLATION*an->cost_to_go;

  aa->root_node = an;

  an->location_next = aa->location_map[ an->location_index ]; // should be NULL
  aa->location_map[ an->location_index ] = an;

  // enqueue root node
  aa->priority_q = an;

  //aa->policy = 0;
  aa->policy = 2;
  aa->best_value = BIG_VALUE;

  aa->expand_next_node_count = 0;

  aa->active = 1;
}

/*******************************************************************/

ASTAR_NODE *handle_duplicates( ASTAR *aa, ASTAR_NODE *an )
{
  ASTAR_NODE *an2, *an3;
  ASTAR_NODE *last_sibling = NULL;
  float d;
  int delete_an = 0;

  if ( an->children != NULL )
  {
    fprintf( stderr, "an_child should be childless at this point.\n" );
    exit( -1 );
  }

  for ( an2 = aa->location_map[ an->location_index ]; an2 != NULL;
      an2 = an2->location_next )
  {
          // printf( "hd: %d %d\n", an->id, an2->id );
          d = state_distance( aa, an->state, an2->state );
          if ( d < DUPLICATE_DISTANCE_THRESHOLD )
            {
                      if ( an->cost_from_start < an2->cost_from_start )
                      { // should replace parent pointer in an2's children with an1
                 // so they pick the better route.
                        last_sibling = NULL;
                        for ( an3 = an2->children; an3 != NULL; an3 = an3->sibling )
                        {
                                  an3->parent = an;
                                  last_sibling = an3;
                        }
                        if ( last_sibling != NULL )
                        {
                                  if ( last_sibling->sibling != NULL )
                                  {
                                 fprintf( stderr,
                                        "last_sibling->sibling should be NULL at this point.\n" );
                                 exit( -1 );
                                  }
                                 // just to be consistent, fix children pointers
                                 last_sibling->sibling = an->children;
                                 an->children = an2->children;
                                 an2->children = NULL;
                        }
                        else
                        {
                                 if ( an2->children != NULL )
                                 {
                                 fprintf( stderr,
                                        "an2->children should be NULL at this point.\n" );
                                 exit( -1 );
                                 }
                        }

                        // SHOULD UPDATE an->children costs and priorities
                        // ideally should reorder priority q, but too expensive

                        /*
                if ( an2->cost_from_start - an->cost_from_start > 0.1
                && DO_PRINTF )
                printf(
                "%g %g XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",
                an->cost_from_start, an2->cost_from_start );
                */
                        delete_an = -1;
                      }
                      else if ( an->children != NULL )
                      {
                        // so this node was better than someone else in the queue.
                        // but is worse than this one.
                        // Do nothing
                        if ( delete_an > -1 )
                        {
                         fprintf( stderr, "Assumption violoation 18238\n" );
                         exit( -1 );
                        }
                      }
                      else
                      {
                        if ( delete_an == 0 )
                                  delete_an = 1;
                      }
            }
  }

  if ( delete_an == 1 )
  {
    if ( an->parent != NULL || an->children != NULL
        || an->sibling != NULL || an->next != NULL
        || an->location_next != NULL )
    {
      fprintf( stderr, "ASTAR_NODE in use 1: %p, %p, %p, %p %p\n",
          (void *) (an->parent), (void *) (an->children),
          (void *) (an->sibling), (void *) (an->next),
          (void *) (an->location_next) );
      exit( -1 );
    }
    free_astar_node( aa, an );
    return NULL;
  }

  an->location_next = aa->location_map[ an->location_index ];
  aa->location_map[ an->location_index ] = an;

  return an;
}

/*******************************************************************/

void next_target_location(ASTAR *aa, ASTAR_NODE *an, ASTAR_NODE *an_child, float x_offset, float y_offset, float rotation){
  int i, j, k, a, n, count,x_off,y_off,a_off;
  int index;
  float s, c, d;
  float foot_x, foot_y;
  float delta_x, delta_y;
  float target_x, target_y, target_z;
  float xx, yy; // spine of search
  float x, y; // actual search
  int found_it = 0;
  float temp_x, temp_y, best_x, best_y, best_z, best_value, ori_shift,temp;
  float value;
  float actual_step_length = STEP_LENGTH;
  float ix, iy;
  float c_a, c_s[2];
  float terrain_feedback[2];
  float foot_pitch, final_foot_pitch;

  // figure out the distance to goal
  c = aa->tt->goal[XX] - an->com[XX];
  s = aa->tt->goal[YY] - an->com[YY];
  d = sqrtf( c*c + s*s );

  if ( d < STEP_LENGTH/2 )
  { // Too close, don't do anything
    // hack to kill child
    an_child->state[XX] = aa->tt->min[XX] - 1;
    return;
  }

  // Get the supporting foot location
  foot_x = an->state[XX];
  foot_y = an->state[YY];

  c_a = an->state[ANGLE];

  c_s[XX] = x_offset*cosf(c_a)-y_offset*sinf(c_a);
  c_s[YY] = x_offset*sinf(c_a)+y_offset*cosf(c_a);


  best_x = xx = target_x = foot_x + c_s[XX];
  best_y = yy = target_y = foot_y + c_s[YY];

  Get_terrain_cost(aa, an, xx, yy, rotation,terrain_feedback, &foot_pitch);

  best_value = terrain_feedback[0];
  best_z = target_z = terrain_feedback[1];
  ori_shift = rotation;
  final_foot_pitch = foot_pitch;
  temp = best_value;


  if(best_value>1000){
    for(x_off=-1;x_off<2;x_off++){
      for(y_off=-1;y_off<2;y_off++){
        for(a_off=-1;a_off<2;a_off++){
          temp_x = target_x+2*DISTANCE*x_off;
          temp_y = target_y+2*DISTANCE*y_off;
          if ( temp_x < aa->tt->min[XX]
              || temp_x > aa->tt->max[XX]
              || temp_y < aa->tt->min[YY]
              || temp_y > aa->tt->max[YY] )
            continue;
          Get_terrain_cost(aa, an, temp_x, temp_y, rotation+(float)(a_off)*3.1415/18.0f, terrain_feedback, &foot_pitch);
         temp = terrain_feedback[0];
          if (temp<best_value){
            best_value = temp;
            best_x = temp_x;
            best_y = temp_y;
         best_z = terrain_feedback[1];
         final_foot_pitch = foot_pitch;        
            ori_shift = rotation+(float)(a_off)*3.1415/18.0f;
          }
          if (temp<1.0)
            break;
        }
        if (temp<1.0)
          break;
      }
      if (temp<1.0)
        break;
    }
    target_x = best_x;
    target_y = best_y;
    target_z = best_z;        
    //if (temp<0.5)
    //fprintf( stderr, "good change from: x %1.3f y %1.3f and a %1.3f to x %1.3f y %1.3f and a %1.3f \n", xx, yy, rotation, target_x, target_y,ori_shift);

    rotation = ori_shift;

  }        


  an_child->state[XX] = target_x;
  an_child->state[YY] = target_y;
  an_child->state[ZZ] = target_z;
  an_child->state[PITCH] = final_foot_pitch;
/*
if (target_z < 0.1)
        an_child->state[ZZ] = 0;
else if (target_z < 0.2)
        an_child->state[ZZ] = 0.16;
else if (target_z < 0.4)
        an_child->state[ZZ] = 0.32;
else
        an_child->state[ZZ] = 0.48;
*/

  an_child->terrain_cost = best_value;

  an_child->com[XX] = (an->state[XX] + an_child->state[XX])/2;
  an_child->com[YY] = (an->state[YY] + an_child->state[YY])/2;

  an_child->com[ANGLE] = an->com[ANGLE] + rotation;
  if (an_child->com[ANGLE] > 2*M_PI)
    an_child->com[ANGLE] = an_child->com[ANGLE] - 2*M_PI;
  else if (an_child->com[ANGLE] < 0)
    an_child->com[ANGLE] = an_child->com[ANGLE] + 2*M_PI;

  an_child->state[ANGLE] = an_child->com[ANGLE];

}

/*******************************************************************/

ASTAR_NODE *generate_16_children( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  ASTAR_NODE *an_child;

  // stop after 12
  if ( *i_child > 11 || *i_child < 0 )
  {
    *i_child = -1;
    return NULL;
  }

  an_child = create_astar_node( aa );
  an_child->depth = an->depth + 1;
  an_child->action_index = *i_child;
  if ( an->state[SIDE] < 0.1 )
    an_child->state[SIDE] = RIGHT;
  else
    an_child->state[SIDE] = LEFT;

  if (*i_child < 12)
  {
    if (an->state[SIDE] == LEFT)        
      next_target_location( aa, an, an_child, action16_l[*i_child][0], action16_l[*i_child][1], action16_l[*i_child][2]);
    else
      next_target_location( aa, an, an_child, action16_r[*i_child][0], action16_r[*i_child][1], action16_r[*i_child][2]);
  }
  else{
    fprintf( stderr, "unknown n_child: %d\n", *i_child );
    exit( -1 );
  }

  (*i_child)++;

  return an_child;
}

/*******************************************************************/

ASTAR_NODE *generate_child( ASTAR *aa, ASTAR_NODE *an, int *i_child )
{
  int index;
  ASTAR_NODE *an_child;

  // printf( "gnn: %d %d\n", an->id, *i_child );

  // allow different search policies
/*
if ( aa->policy == 0 )
{
an_child = generate_4_children( aa, an, i_child );
}
else if ( aa->policy == 1 )
{
// allow different numbers of actions at different steps
if ( an->depth == 0 )
an_child = generate_8_children( aa, an, i_child );
else
an_child = generate_4_children( aa, an, i_child );
}
else
{
// allow different numbers of actions at different steps
if ( an->depth == 0 )
an_child = generate_16_children( aa, an, i_child );
else if ( an->depth == 1 )
an_child = generate_8_children( aa, an, i_child );
else
an_child = generate_4_children( aa, an, i_child );
}
*/

  // use single search police

  an_child = generate_16_children( aa, an, i_child );

  if ( an_child == NULL )
  {
    //printf( "gnn: NULL child 1 \n" );
    return NULL;
  }

  //printf( "gnn: %d %d %d\n", an->id, an_child->id, *i_child );

  // out of bounds? This is useful, but also a hack to reject failed tries
  if ( an_child->state[XX] < aa->tt->min[XX]
      || an_child->state[XX] > aa->tt->max[XX]
      || an_child->state[YY] < aa->tt->min[YY]
      || an_child->state[YY] > aa->tt->max[YY]
      || an_child->com[XX] < aa->tt->min[XX]
      || an_child->com[XX] > aa->tt->max[XX]
      || an_child->com[YY] < aa->tt->min[YY]
      || an_child->com[YY] > aa->tt->max[YY] )
  {
         if ( an_child->parent != NULL || an_child->children != NULL
                || an_child->sibling != NULL || an_child->next != NULL
                || an_child->location_next != NULL )
         {
         fprintf( stderr, "ASTAR_NODE in use 2: %p, %p, %p, %p %p\n",
                 (void *) (an->parent), (void *) (an->children),
                 (void *) (an->sibling), (void *) (an->next),
                 (void *) (an->location_next) );
         exit( -1 );
         }

         free_astar_node( aa, an_child );
         //printf( "gnn: child state %g %g %g %g \n", an_child->state[XX], an_child->state[YY], an_child->com[XX], an_child->com[YY]);
         return NULL;
  }


  terrain_indices( aa->tt, an_child->state[XX], an_child->state[YY],
      NULL, NULL, &index );
  if ( index < 0 || index >= aa->tt->n_cells )
  {
    fprintf( stderr, "State out of bounds %d %d; %g %g %g %g %g %g %g\n", index,
        aa->tt->n_cells, an_child->state[XX], an_child->state[YY],
        an_child->state[ANGLE], aa->tt->min[XX], aa->tt->min[YY], aa->tt->max[XX],aa->tt->max[YY] );
    exit( -1 );
  }
  // currently all of these are the same, but eventually they may be different
  an_child->terrain_index = index;
  an_child->location_index = index;


  an_child->one_step_cost = astar_step_cost( aa, an_child, an);


  an_child->cost_from_start = an->cost_from_start
    + an_child->one_step_cost + an_child->terrain_cost;

  an_child->cost_to_go = heuristic( aa, an_child );


  an_child->priority
    = an_child->cost_from_start + INFLATION*an_child->cost_to_go;
  // could implement a too expensive limit (although this just goes on
  // end of prioity queue)

  // printf( "gnn: before hd: %d %d %d\n", an->id, an_child->id, *i_child );

  // check for duplicates
  an_child = handle_duplicates( aa, an_child );

  if ( an_child == NULL )
  {
    //printf( "gnn: NULL child 3 \n" );        
    return NULL;
  }

  //printf( "gnn: after hd: %d %d %d\n", an->id, an_child->id, *i_child );

  an_child->parent = an;
  an_child->sibling = an->children;
  an->children = an_child;

  return an_child;
}

/*******************************************************************/

void enqueue( ASTAR *aa, ASTAR_NODE *an )
{
  int j;
  ASTAR_NODE **prev;
  ASTAR_NODE *q;

  if ( an == NULL )
    return;

  if ( aa->priority_q == NULL )
  {
    /*
fprintf( stderr, "NULL Priority Queue\n" );
exit( -1 );
*/
    aa->priority_q = an;
    return;
  }

  prev = &(aa->priority_q);
  q = aa->priority_q;

  for ( j = 0; ; j++ )
  {
    if ( q == NULL )
    {
      an->next = q;
      *prev = an;
      return;
    }

    if ( q->priority >= an->priority )
    {
      an->next = q;
      *prev = an;
      return;
    }

    prev = &(q->next);
    q = q->next;

    if ( j > 1000000 )
    {
      fprintf( stderr, "enqueue: loop broken.\n" );
      exit( -1 );
    }
  }
}

/*******************************************************************/

void printf_priority_q( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->priority_q;
  // printf( "priority_queue: " );
  for ( ; ; )
  {
    if ( an == NULL )
      break;
    // printf( "%d, ", an->id );
    count++;
    an = an->next;
  }
  // printf( "\n" );
  printf( "Priority queue: %d nodes.\n", count );
}

/*******************************************************************/

void printf_done_list( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->done_list;
  // printf( "done_list: " );
  for ( ; ; )
  {
    if ( an == NULL )
      break;
    // printf( "%d, ", an->id );
    count++;
    an = an->next;
  }
  // printf( "\n" );
  printf( "Done list: %d nodes.\n", count );
}

/*******************************************************************/

void printf_at_goal( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->at_goal;
  // printf( "at_goal_list: " );
  for ( ; ; )
  {
    if ( an == NULL )
      break;
    // printf( "%d, ", an->id );
    count++;
    an = an->next;
  }
  // printf( "\n" );
  printf( "At_goal list: %d nodes.\n", count );
}

/*******************************************************************/

void printf_free_list( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->free_list;
  // printf( "free_list: " );
  for ( ; ; )
  {
    if ( an == NULL )
      break;
    // printf( "%d, ", an->id );
    count++;
    an = an->next;
  }
  // printf( "\n" );
  printf( "Free list: %d nodes. %d ids\n", count, astar_node_count );
}

/*******************************************************************/

void printf_path( ASTAR *aa )
{
  ASTAR_NODE *an;
  int count = 0;

  if ( !DO_PRINTF )
    return;

  an = aa->at_goal;
  printf( "path:\n" );
  for ( ; ; )
  {
    if ( an == NULL )
      break;
    printf( "%d %d: %g %g %g %g; %g %g %g %g\n",
        an->id, an->depth, an->state[XX], an->state[YY],
        an->state[ANGLE], an->state[SIDE],
        an->cost_from_start, an->cost_to_go, an->priority,
        an->terrain_cost );
    count++;
    an = an->parent;
  }
  // printf( "path: %d nodes.\n", count );
}

/*******************************************************************/

void printf_final_path( PATH_NODE *pn )
{
  int count = 0;
  PATH_NODE *pn_original;

  pn_original = pn;

  if ( DO_PRINTF )
    printf( "final path from goal:\n" );
  for ( ; ; )
  {
    if ( pn == NULL )
      break;
    if ( DO_PRINTF )
      printf( "%3d: %7.3f %7.3f %7.3f %7.3f %7.3f %2.0f %3d; %8.3f %8.3f %8.3f %8.3f %8.3f\n",
          pn->id, pn->state[XX], pn->state[YY], pn->state[ZZ], pn->state[ANGLE], pn->state[PITCH],
          pn->state[SIDE], pn->action_index,
          pn->cost_from_start, pn->cost_to_go, pn->priority,
          pn->one_step_cost,
          pn->terrain_cost );
    count++;
    pn = pn->previous;
  }
  printf( "Final path: cost %g; %d nodes.\n", pn_original->cost_from_start, count );
}

/*******************************************************************/

void expand_next_node( ASTAR *aa, ASTAR_NODE *an )
{
  int i;
  int j;
  ASTAR_NODE *an2;
  ASTAR_NODE *child;

  if ( an == NULL )
    return;

  if ( debug_a )
    printf( "expand_next_node: %d %g %g\n", an->id,
        an->state[XX], an->state[YY] );

  for ( i = 0; ; )
  {
    // printf( "enn: %d %d\n", an->id, i );

    // stop making children
    if ( i < 0 )
      break;
    
    child = generate_child( aa, an, &i );

    //printf( "ennx: %d %d\n", an->id, i );

    if ( child == NULL )
      continue;

    if ( child->next != NULL )
    {
      fprintf( stderr, "Expected child's next to be NULL\n" );
      exit( -1 );
    }

    if ( ( aa->tt->known_terrain[ child->terrain_index ] < 0.1 )
        // kill stuff that is in unknown territory away from goal
        && ( child->cost_to_go > aa->root_node->cost_to_go ) )
    {
      child->next = aa->done_list;
      aa->done_list = child;
    }
    else if ( child->cost_to_go < CLOSE_ENOUGH )
    {
      if ( aa->at_goal == NULL )
        printf( "enn: %d %g\n", aa->expand_next_node_count,
            child->priority );
      else if ( aa->at_goal->priority > child->priority )
        printf( "enn: %d %g %g\n", aa->expand_next_node_count,                
            aa->at_goal->priority, child->priority );

      // manage best so far
      if ( aa->best_value > child->priority )
        aa->best_value = child->priority;

      if ( aa->at_goal == NULL )
      {
        // printf( "enn: NULL at_goal\n" );
        child->next = NULL;
        aa->at_goal = child;
      }
      else if ( aa->at_goal->priority > child->priority )
      {
        /*
printf( "enn: new guy better: %g %g\n", child->priority,
aa->at_goal->priority );
*/
        child->next = aa->at_goal;
        aa->at_goal = child;
      }
      else
      {
        an2 = aa->at_goal;
        for( j = 0; ; j++ )
        {
          /*
if ( an2->next == NULL )
printf( "an2: %d %d\n", an2->id, j );
else
printf( "an2: %d %d %d\n", an2->id, an2->next->id, j );
*/
          if ( an2->next == NULL)
          {
            // printf( "enn: last guy in queue\n" );
            an2->next = child;
            break;
          }
          if ( an2->next->priority > child->priority )
          {
            /*
printf( "enn: new guy better: %g %g\n", child->priority,
an2->next->priority );
*/
            child->next = an2->next;
            an2->next = child;
            break;
          }
          an2 = an2->next;

          if ( j > 1000000 )
          {
            fprintf( stderr, "enn: loop broken.\n" );
            exit( -1 );
          }
        }
      }
      if ( QUIT_ON_FIRST_SUCCESS )
      {
        aa->done = 1;
        if ( DO_PRINTF )
        {
          /*
printf( "Done.\n" );
printf_priority_q( aa );
printf_done_list( aa );
printf_path( aa );
*/
        }
        return;
      }
    }
    else
    {
      enqueue( aa, child );
    }
  }

  if ( debug_a )
    printf( "expand_next_node done: %d\n", an->id );

  /*
displaya();
glutSetWindow( wa );
glutPostRedisplay();
printf( "press\n" );
getchar();
*/
}

/******* PATH ****************************************************/

PATH_NODE* create_path_node( ASTAR_NODE *an )
{
  int i;
  PATH_NODE *pn;
  static int path_node_count = 0;

  pn = (PATH_NODE *) malloc( sizeof( PATH_NODE ) );

  if ( pn == NULL )
  {
    fprintf( stderr, "Can't allocate path node.\n" );
    exit( -1 );
  }
  pn->id = path_node_count++;
  pn->an_id = an->id;
  pn->action_index = an->action_index;
  for ( i = 0; i < N_X; i++ )
    pn->state[i] = an->state[i];
  pn->previous = NULL;
  pn->next = NULL;
  pn->cost_from_start = an->cost_from_start;
  pn->cost_to_go = an->cost_to_go;
  pn->one_step_cost = an->one_step_cost;
  pn->terrain_cost = an->terrain_cost;
  pn->priority = an->priority;

  return pn;
}

/*******************************************************************/

void add_an_to_path( ASTAR *aa, ASTAR_NODE *an )
{
  PATH_NODE *pn;

  pn = create_path_node( an );

  pn->previous = p1;
  if ( p1 != NULL )
  {
    p1->next = pn;
    //pn->cost_from_start += p1->cost_from_start;
  }
  p1 = pn;

}

/******* PERCEPTION ****************************************************/

void perceive( TERRAIN *tt )
{
  int ixc, iyc;
  int ix, iy;
  float value;

  terrain_indices( tt, tt->current[XX], tt->current[YY], &ixc, &iyc, NULL );

  for ( ix = ixc - tt->perception_radius;
      ix - ixc <= tt->perception_radius;
      ix++ )
  {
    for ( iy = iyc - tt->perception_radius;
        iy - iyc < tt->perception_radius;
        iy++ )
    {
      if ( ((ix-ixc)*(ix-ixc) + (iy-iyc)*(iy-iyc)) <
          (tt->perception_radius)*(tt->perception_radius) )
      {
        value = get_true_cost_map_pixel( tt, ix, iy );
        set_perceived_cost_map_pixel( tt, ix, iy, value );
      }
    }
  }
}

/******* PLANNING ****************************************************/

int plan_count = 0;

void plan()
{
  int j, temp;
  ASTAR_NODE *an, *an_temp, *an_temp2;
  int first_time = 1;
  clock_t start, end;

  printf( "plan: %d\n", ++plan_count );
  free_astar( a1 );
  load_actions();        
  perceive( t1 );
  init_astar( a1 );


  start = clock();
 
  for( a1->expand_next_node_count = 0; a1->expand_next_node_count < 100000;
      a1->expand_next_node_count++ )
  {
    
    //printf( "count No: %d\n", a1->expand_next_node_count );
    if ( a1->done ){
      printf( "End 1\n");
      break;        
    }

    end = clock();
    if ( (end-start)/(1000) > 20000.0 ){
      printf( "End 3\n");
      break;        
    }

    if ( a1->priority_q == NULL )
    {
      // a1->done = 1;
      printf( "NULL priority_q: %d\n", a1->expand_next_node_count );
      a1->policy++;
      if ( a1->policy >= 3)
      {
        a1->done = 1;
        printf( "End 2\n");
        break;
      }
      else
        expand_next_node( a1, a1->root_node );
      continue;
    }

    an = a1->priority_q;
    a1->priority_q = an->next;
    an->next = a1->done_list;
    a1->done_list = an;


    if ( an->priority > a1->best_value + CUTOFF_INCREMENT )
    {
      // this node doesn't count
      a1->expand_next_node_count--;
      continue;
    }

    expand_next_node( a1, an );
  }
  printf( "%d node expansions.\n", a1->expand_next_node_count );

  if ( debug_plan )
    printf( "p120 %d\n", plan_count );


  // Okay, use best termination
  if ( a1->at_goal ) // this is good
  {
    an = a1->at_goal;
    if ( DO_PRINTF )
      printf( "****GOOD****** planning using at_goal node\n" );
  }
  else if ( a1->priority_q ) // this is less good
  {
    an = a1->priority_q;
    if ( DO_PRINTF )
      printf( "****OK******* planning using priority_q node\n" );
  }
  else
  {
    an = a1->done_list; // this is really bad
    if ( DO_PRINTF )
      printf( "****BAD******* planning using done_list node\n" );
  }

  if ( an == NULL )
  {
    fprintf( stderr, "Null search.\n" );
    exit( -1 );
  }

  if ( debug_plan )
    printf( "p130 %d\n", plan_count );

  // Find starting step
  for ( j = 0; ; j++ )
  {
    if ( an->parent == NULL ) // this is really really bad, 1 node only
    {
      path_done = 1;
      if ( DO_PRINTF )
        printf( "Unable to make a plan that takes 1 step.\n" );
      return; // we give up
    }
    if ( an->parent->parent == NULL )
      break;
    add_an_to_path( a1, an );
    an = an->parent;
    if ( j > 1000000 )
    {
      fprintf( stderr, "plan: loop broken.\n" );
      fprintf( stderr, "plan: it ded.\n" );
      exit( -1 );
    }
  }

  add_an_to_path( a1, an );

  if ( first_time )
  {
    if ( an->parent == NULL )
    {
      fprintf( stderr, "Null parent???\n" );
      exit( -1 );
    }
    add_an_to_path( a1, an->parent );
  }

  first_time = 0;

  if ( debug_plan )
    printf( "p200 %d\n", plan_count );
}



