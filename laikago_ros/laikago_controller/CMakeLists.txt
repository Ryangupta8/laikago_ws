cmake_minimum_required(VERSION 2.8.3)
project(laikago_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    laikago_msgs
    laikago_gazebo
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    pluginlib
    nodelet
    control_toolbox
)

catkin_package(
    CATKIN_DEPENDS
    laikago_msgs
    laikago_gazebo
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    roscpp
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


# Declare a C++ library
add_library(${PROJECT_NAME}
    ../laikago_gazebo/src/body.cpp 
)

link_directories($(catkin_LIB_DIRS) lib)

add_library(laikago_joint_controller src/joint_controller.cpp)
add_dependencies(laikago_joint_controller laikago_msgs_gencpp)
target_link_libraries(laikago_joint_controller ${catkin_LIBRARIES} laikago_control_tool)


add_executable(test_gazebo_control src/test_gazebo_control.cpp)
add_dependencies(test_gazebo_control laikago_msgs_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_gazebo_control ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_joint_control src/test_joint_control.cpp)
add_dependencies(test_joint_control laikago_joint_controller laikago_msgs_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_joint_control ${PROJECT_NAME} ${catkin_LIBRARIES} laikago_joint_controller laikago_control_tool)

 
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
