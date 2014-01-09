cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(/usr/include/vtk-5.8)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_add_executable(cloud_assembler src/cloud_assembler.cpp)
rosbuild_link_boost(cloud_assembler thread)
rosbuild_add_executable(cloud_assembler_ocu src/cloud_assembler_ocu.cpp)
rosbuild_link_boost(cloud_assembler_ocu thread)

rosbuild_add_executable(foot_interface_all src/foot_interface_all.cpp)
rosbuild_link_boost(foot_interface_all thread)
rosbuild_add_executable(foot_interface_ocu src/foot_interface_ocu.cpp)
rosbuild_link_boost(foot_interface_ocu thread)

rosbuild_add_executable(step_plan_all src/map_plan_all.cpp src/astar.cpp)
rosbuild_link_boost(step_plan_all thread)
rosbuild_add_executable(step_plan_ocu src/map_plan_ocu.cpp src/astar.cpp)
rosbuild_link_boost(step_plan_ocu thread)



target_link_libraries(cloud_assembler libvtkCommon.so libvtkFiltering.so libvtkRendering.so libboost_signals.so ${PCL_LIBRARIES})
target_link_libraries(cloud_assembler_ocu libvtkCommon.so libvtkFiltering.so libvtkRendering.so libboost_signals.so ${PCL_LIBRARIES})
target_link_libraries(step_plan_all libvtkCommon.so libvtkFiltering.so libvtkRendering.so libboost_signals.so ${PCL_LIBRARIES})
target_link_libraries(step_plan_ocu libvtkCommon.so libvtkFiltering.so libvtkRendering.so libboost_signals.so ${PCL_LIBRARIES})


