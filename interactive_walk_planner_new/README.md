# Interactive walking planner for Atlas control

####Package name
interactive_walk_planner

####Description
launch file (under launch folder):
1: interactive_walk_planner.launch use for simulation
2: interactive_walk_planner_online.launch use for real Atlas control

main nodes:
cloud_assembler: it assemble the laser scan data to point cloud. It is
similar as wrecs/laser/points2, but with /l_foot as reference frame. My
planner is working in this frame.
foot_interface: it handle all the interactive foot markers and send the
final food sequences to the robot.
step_plan: it builds the terrain map and plans the foot sequence according
to the terrain map.

foot_interface_online and step_plan_online are for real Atlas control

####Running the Package
Run RVIZ RVIZ
RVIZ setting:
1. set the fixed frame to "/l_foot"
2. add a pointcloud2 listen to topic "/filtered_cloud"
3. add a interactivemarker listen to topic "/foot_interface/update"

bend the head down to 0.4~0.5 rad to have a better point cloud view

All the user interface and control are done in RVIZ. Here are some lists actions:

1. give the goal location by move the interactive marker
2. send the goal location to planner by click the manu marker (select the
first one)
3. the planner will generate the foot sequence and show it on the RVIZ
4. The new foot sequence can be modified also on the RVIZ
5. If the foot sequence is not we want, we can also send the request to
redo the planning (click manu marker select the second one and resent the
new goal location)
6. If everything are satisfied, we can send the foot sequence to the ATLAS
(click manu marker select the third one)
