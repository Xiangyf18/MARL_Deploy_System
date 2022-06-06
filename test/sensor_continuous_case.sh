#!/bin/bash
num_agents=1
robot_type='Mecanum' #Ackerman
cur_path=$(dirname "$0")
#********************************************************************
#you should execute "deploy_clients.sh" first on Master Computer
#and then, execute this shell script on each robot
#********************************************************************


# 服务器上的reset指令备注，给zsq
# cd /home/ubuntu/project/sim2real/sim2real/hosts/host_backend/scripts
# ./deploy_clients.sh  Mecanum 1 depth_camera

source ${cur_path}/../scripts/setup_localhost.sh
#args: --lidar_on --camera_on
python3 ${cur_path}/sensor_continuous_case.py    --num_agents ${num_agents} --robot_type ${robot_type} --depth_camera_on


