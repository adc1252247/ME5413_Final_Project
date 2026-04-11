cmake_minimum_required(VERSION 3.0.2)
project(me5413_group9)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



find_package(catkin REQUIRED COMPONENTS
  cv_bridge
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



set(COMMON_SOURCES
  src/common/template_matching.cpp
  src/common/robot_control.cpp
)



add_executable(main src/main.cpp ${COMMON_SOURCES})
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(cv_control src/cv_control.cpp ${COMMON_SOURCES})
target_link_libraries(cv_control ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(points_to_scan src/points_to_scan.cpp ${COMMON_SOURCES})
target_link_libraries(points_to_scan ${catkin_LIBRARIES} ${OpenCV_LIBS})




file(GLOB_RECURSE cmake_files "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cmake")
list(SORT cmake_files)

message(STATUS "Files: ${cmake_files}")

foreach(cmake_file ${cmake_files})
    if(EXISTS ${cmake_file})
        message(STATUS "Including: ${cmake_file}")
        include(${cmake_file})
    else()
        message(WARNING "CMake file not found: ${cmake_file}")
    endif()
endforeach()