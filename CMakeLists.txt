cmake_minimum_required(VERSION 2.8.3)
project(gap_estimation)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  dynamic_reconfigure
)

find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")


generate_dynamic_reconfigure_options(
    cfg/gap_estimation.cfg
)

include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gap_estimation_lib
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs rospy roscpp std_msgs tf2_ros tf2_sensor_msgs visualization_msgs dynamic_reconfigure
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

target_include_directories(gap_estimation PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

target_link_libraries(gap_estimation
  gap_estimation_lib
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
