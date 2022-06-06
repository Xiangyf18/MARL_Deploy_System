#!/bin/bash
num_agents=1
robot_type='Mecanum' #Ackerman
cur_path=$(dirname "$0")
#********************************************************************
#you should execute "deploy_clients.sh" first on Master Computer
#and then, execute this shell script on each robot
#********************************************************************



# deploy system ,for example
#args: lidar camera
#source ${cur_path}/../scripts/deploy_clients.sh $robot_type $num_agents  


source ${cur_path}/../scripts/setup_localhost.sh
#args: --lidar_on --camera_on
python3 ${cur_path}/distributed_case.py    --num_agents ${num_agents} --robot_type ${robot_type}


