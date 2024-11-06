cmake_minimum_required(VERSION 3.0.2)
project(visual_sonar_rgb_mapping)

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf2_ros
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar_mapping
  LIBRARIES Eigen3 PCL OpenCV
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_map_drawer src/MapDrawer.cpp)
target_link_libraries(${PROJECT_NAME}_map_drawer
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_map src/Map.cpp)
target_link_libraries(${PROJECT_NAME}_map
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_mappoint src/MapPoint.cpp)
target_link_libraries(${PROJECT_NAME}_mappoint
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_frame src/Frame.cpp)
target_link_libraries(${PROJECT_NAME}_frame
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(mapping
  src/main.cpp
)
target_link_libraries(mapping
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_map_drawer
  ${PROJECT_NAME}_map
  ${PROJECT_NAME}_mappoint
  ${PROJECT_NAME}_frame
)