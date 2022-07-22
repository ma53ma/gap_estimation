cmake_minimum_required(VERSION 2.8.3)
project(gap_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  dynamic_reconfigure
)

# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")


generate_dynamic_reconfigure_options(
    cfg/gap_estimation.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gap_estimation_lib
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros pcl_conversions sensor_msgs tf2_ros tf2_sensor_msgs visualization_msgs dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target names are not consistent with filenames to avoid breaking api changes
add_library(gap_estimation_lib 
	    	src/gap_estimation.cpp
	    	src/cart_model.cpp
		src/gap_associator.cpp
		src/gap_estimator_config.cpp
		src/gap_utils.cpp
		src/visualization.cpp)

add_dependencies(gap_estimation_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(gap_estimation_lib PUBLIC -std=c++11)
target_link_libraries(gap_estimation_lib 
  ${catkin_LIBRARIES}
)

add_executable(gap_estimation src/gap_estimation_node.cpp)
add_dependencies(gap_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(gap_estimation PUBLIC -std=c++11)

# target_include_directories(gap_estimation PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

target_link_libraries(gap_estimation
  gap_estimation_lib
  ${catkin_LIBRARIES}
)
