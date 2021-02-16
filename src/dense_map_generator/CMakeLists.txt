cmake_minimum_required(VERSION 2.8.3)
project(dense_map_generator)

set(CMAKE_BUILD_TYPE "RELWITHDEBINFO")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  message_generation
  tf_conversions
)

catkin_package(CATKIN_DEPENDS
        sensor_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(dense_map_generator
     src/dense_map_generator.cpp
)

target_link_libraries(dense_map_generator
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(
  TARGETS
  dense_map_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
