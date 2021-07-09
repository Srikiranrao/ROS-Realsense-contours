# ROS-Realsense-contours
ros_realsense_image
----------------------
This repo subscribes to realsense ros topics and generates contours.

Contours gives X,Y center point and angle of objects
 

Environment Requirements:
--------------------------------------
Environments : Ubuntu 20.04 and install opencv and ROS noetic



--------------------------------------
Terminal 1 :
run realsense ROS package 
package can be downloaded from below repo

https://github.com/IntelRealSense/realsense-ros

Terminal 2 :
Run image subscriber to receive two topics: /camera/rgbd/image and /camera/depth/image_raw

`$ rosrun ros_realsense_image image_realsense_subscriber`

