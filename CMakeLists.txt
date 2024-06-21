cmake_minimum_required(VERSION 3.13)
project(pcr)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  eigen_conversions
  message_generation
  cv_bridge

  # nano_gicp
)

find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

add_message_files(
        FILES
        cloud_info.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

include(3rdparty/find_dependencies.cmake)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

set(fpfh_src
    src/fpfh/feature_matcher.cpp 
    src/fpfh/fpfh.cpp
)
set(quatro_src
    src/quatro/graph.cpp
)

set(teaser_src
    src/teaser/graph.cpp
    src/teaser/teaser.cpp
)

add_executable(${PROJECT_NAME} 
               ${fpfh_src}
               ${quatro_src}
               ${teaser_src}
               src/fitter.cpp
               src/register.cpp
               src/main.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} 
                      ${PCL_LIBRARIES} 
                      ${PYTHON_LIBRARIES}
                      stdc++fs
                      pmc::pmc
)

target_include_directories(${PROJECT_NAME} 
                            PRIVATE 
                            ${PYTHON_INCLUDE_DIRS}
)