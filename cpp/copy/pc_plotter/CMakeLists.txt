cmake_minimum_required(VERSION 3.0.2)
project(pc_plotter)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(catkin REQUIRED
    roscpp
    rosbag
)
find_package(PythonLibs)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

add_executable(num_points_plotter src/num_points_plotter.cpp)
target_link_libraries(num_points_plotter
    ${catkin_LIBRARIES}
    ${PYTHON_LIBRARIES}
)

add_executable(pc_plotter src/pc_plotter.cpp)
target_link_libraries(pc_plotter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PYTHON_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############