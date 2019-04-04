cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(pointcloud_intensity)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  rospy  
  roscpp
  sensor_msgs
)


catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs )


find_package(Eigen3  REQUIRED)
find_package(Boost  REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(intensity_test src/main.cc) 
target_link_libraries(intensity_test ${catkin_LIBRARIES})

