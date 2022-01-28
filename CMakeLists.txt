cmake_minimum_required(VERSION 3.0.2)
project(hydra_demos)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    descartes_utilities
    trajectory_msgs
    eigen_conversions
    gcode_core
)
catkin_package(CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    descartes_core
    descartes_moveit
    descartes_trajectory
    descartes_planner
    descartes_utilities
    trajectory_msgs
    eigen_conversions
    gcode_core
    xacro
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_moveit_demo src/moveit_demo.cpp)
target_link_libraries(${PROJECT_NAME}_moveit_demo ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_moveit_multi_demo src/moveit_multi_demo.cpp)
target_link_libraries(${PROJECT_NAME}_moveit_multi_demo ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_planners src/cartesian_planner.cpp)
target_link_libraries(${PROJECT_NAME}_planners ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_planners PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_planners SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_descartes_demo src/descartes_demo.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_demo ${PROJECT_NAME}_planners ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_descartes_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_descartes_demo SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_descartes_demo_node src/descartes_demo_node.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_demo_node ${PROJECT_NAME}_descartes_demo ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_descartes_demo_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_descartes_demo_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_descartes_multi_demo src/descartes_multi_demo.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_multi_demo ${PROJECT_NAME}_planners ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_descartes_multi_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_descartes_multi_demo SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_descartes_multi_demo_node src/descartes_multi_demo_node.cpp)
target_link_libraries(${PROJECT_NAME}_descartes_multi_demo_node ${PROJECT_NAME}_descartes_multi_demo ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_descartes_multi_demo_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_descartes_multi_demo_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})