cmake_minimum_required(VERSION 2.8.3)
project(ri_om)
add_definitions(-std=c++14) 
set(CMAKE_CXX_STANDARD 14)


SET(CMAKE_BUILD_TYPE "release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


include_directories("/usr/include/eigen3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  tf
  tf_conversions
)

add_message_files(FILES key_info.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS  
  message_generation
  std_msgs 
  message_runtime
  roscpp 
  rospy
  PCL 
  EIGEN3

)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED QUIET)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# build lib
ADD_LIBRARY (scancontext STATIC lib/scancontext/Scancontext.cpp )
target_link_libraries(scancontext  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ADD_LIBRARY (zsy_ndt_lib STATIC lib/scan_to_map.cpp  lib/ndt_map.cpp lib/ros_msg_help.cpp lib/closeloop_detection.cpp lib/common.cpp  )
target_link_libraries(zsy_ndt_lib scancontext ${PCL_LIBRARIES}  ${CERES_LIBRARIES} gtsam)


# main
add_executable(fast_odom src/fast_odom.cpp  )
target_link_libraries(fast_odom zsy_ndt_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${CERES_LIBRARIES})

add_executable(mapping src/mapping.cpp )
target_link_libraries(mapping  zsy_ndt_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${CERES_LIBRARIES})

add_executable(transformfusion src/transform_fusion.cpp )
target_link_libraries(transformfusion  zsy_ndt_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${CERES_LIBRARIES})

