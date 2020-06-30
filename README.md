# laikago_ws

This is the catkin workspace for the Unitree Laikago Quadruped. 
The code in this repo should live in name_of_ws/src. This is built for ROS Melodic on Ubuntu 18.04.

In order to make the RealSense packages work, please see the following links:
1) https://downloadcenter.intel.com/download/29617/Latest-Firmware-for-Intel-RealSense-D400-Product-Family?v=t
2) https://github.com/IntelRealSense/librealsense
3) https://github.com/IntelRealSense/realsense-ros
  a) Note that in order to successfully ``` catkin build ``` these packages, you will need to take a look at this issue:
    https://github.com/IntelRealSense/realsense-ros/issues/821 (see comment by phil-ludewig)


Download the LCM: http://lcm-proj.github.io/

In order to build laikago_ros packages, please do the following:
```cd ~```
```git clone https://github.com/Ryangupta8/laikago_sdk.git```
Then follow the instructions there in that README. In addition, you will need to update the CMakeLists.txt in the laikago_real package to reflect the location of the laikago_sdk folder.

