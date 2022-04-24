cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  geometry_msgs
  tf
  cv_bridge
  eigen_conversions
  message_generation 
  std_srvs
)

add_service_files(
   FILES
   saveMap.srv
   optimizeMap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)
include(cmake/sophus.cmake)
include(cmake/opencv.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

#add_executable(localization_node src/apps/localization_node.cpp ${ALL_SRCS})
#add_dependencies(localization_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(localization_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(docker_node src/apps/docker_node.cpp ${ALL_SRCS})
add_dependencies(docker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(docker_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#add_executable(pcl_save_node src/apps/pcl_save_node.cpp ${ALL_SRCS})
#add_dependencies(pcl_save_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pcl_save_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
