cmake_minimum_required(VERSION 3.0.2)
project(hydra_demos)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    trajectory_msgs
    eigen_conversions
    mrmf_core
)
catkin_package(CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    trajectory_msgs
    eigen_conversions
    mrmf_core
    xacro
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_mrmf_demo_node src/mrmf_demo_node.cpp)
target_link_libraries(${PROJECT_NAME}_mrmf_demo_node ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_mrmf_demo_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_mrmf_demo_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_composite_interpolator_node src/composite_interpolator_node.cpp)
target_link_libraries(${PROJECT_NAME}_composite_interpolator_node ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_composite_interpolator_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_composite_interpolator_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})