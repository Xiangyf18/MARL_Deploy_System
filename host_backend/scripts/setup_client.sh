#!/bin/bash

#catkin_make --only-pkg-with-deps nics_robot_client xtark_driver
source devel/setup.sh
export ROS_MASTER_URI=http://172.16.0.62:11311
export CAR_ID=`python scripts/mac2id.py`
#Ackerman or Macanum
export DRIVER_DIR=$(find ~/ros_ws/src -name "xtark_driver" | find ~/ros_ws/src -name "xtark_ros_wrapper")
chmod +x src/nics_robot_client/scripts/robot_client_node.py

{
    ps -ef | grep client_base.launch | awk '{print $2}' | xargs kill -9
    roslaunch scripts/client_base.launch
} &

if [ $1 == 1 ]
then
{
    ps -ef | grep client_lidar.launch | awk '{print $2}' | xargs kill -9
    roslaunch scripts/client_lidar.launch
} &
fi

if [ $2 == 1 ]
then
{
    ps -ef | grep client_camera.launch | awk '{print $2}' | xargs kill -9
    roslaunch scripts/client_camera.launch
} &

fi

if [ $3 == 1 ]
then
{
    ps -ef | grep client_depth_camera.launch | awk '{print $2}' | xargs kill -9
    roslaunch scripts/client_depth_camera.launch
    # roslaunch xtark_nav_depthcamera  xtark_depthcamera.launch

} &

fi