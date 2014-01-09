cmake_minimum_required(VERSION 2.8)

  include (FindPkgConfig)
  if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_INSTALL_PREFIX $ENV{DRC}/gazebo)

include(cmake/rospack.cmake)
  get_rospack_flags(roscpp)
  get_rospack_flags(image_transport)
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
  set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
include_directories(${image_transport_INCLUDE_DIRS})
#link_directories(${image_transport_LIBRARY_DIRS})
#  set(CMAKE_INSTALL_RPATH "${image_transport_LIBRARY_DIRS}")

add_library(VehiclePlugin_w SHARED VRCPlugin.cpp)
  set_target_properties(VehiclePlugin_w PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
  set_target_properties(VehiclePlugin_w PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(VehiclePlugin_w ${roscpp_LIBRARIES})
install (TARGETS VehiclePlugin_w DESTINATION ${CMAKE_INSTALL_PREFIX}/plugins/)
