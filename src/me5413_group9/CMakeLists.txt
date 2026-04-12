cmake_minimum_required(VERSION 3.0.2)
project(me5413_group9)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roslib
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  actionlib
  actionlib_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES me5413_group9
  CATKIN_DEPENDS cv_bridge roscpp geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS},
  ${OpenCV_INCLUDE_DIRS}
)



# Transform 3D LIDAR to 2D LaserScan
add_executable(laserscan_from_lidar src/laserscan_from_lidar.cpp)
target_link_libraries(laserscan_from_lidar ${catkin_LIBRARIES})

add_executable(main src/main.cpp src/common/template_matching.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Phase 2 executable
add_executable(phase_2_node src/phase_2.cpp)
target_link_libraries(phase_2_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(phase_2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(phase_2_slope_node src/phase_2_slope.cpp)
# target_link_libraries(phase_2_slope_node ${catkin_LIBRARIES})

# add_executable(corridor_navigator src/corridor_navigator.cpp)
# target_link_libraries(corridor_navigator ${catkin_LIBRARIES})

