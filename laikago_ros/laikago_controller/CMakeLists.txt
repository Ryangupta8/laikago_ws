cmake_minimum_required(VERSION 2.8.3)
project(laikago_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    realtime_tools
    laikago_msgs
    laikago_gazebo
)

catkin_package(
    CATKIN_DEPENDS
    laikago_msgs
    laikago_gazebo
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(/home/knapsack/robots/laikago_sdk/include)
link_directories($(catkin_LIB_DIRS) lib)

add_library(laikago_joint_controller src/joint_controller.cpp)
add_dependencies(laikago_joint_controller laikago_msgs_gencpp)
target_link_libraries(laikago_joint_controller ${catkin_LIBRARIES} laikago_control_tool)


add_executable(test_joint_control src/test_joint_control.cpp)
add_dependencies(test_joint_control laikago_joint_controller laikago_controller laikago_gazebo ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_joint_control ${catkin_LIBRARIES})

 
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")