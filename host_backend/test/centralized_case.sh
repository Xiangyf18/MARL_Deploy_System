#!/bin/bash
num_agents=1
robot_type='Mecanum' #Ackerman
cur_path=$(dirname "$0")
# deploy system
#args: lidar camera

source ${cur_path}/../scripts/deploy_clients.sh $robot_type $num_agents  
if [ $? != 0 ]
then
    exit
fi
#args: --lidar_on --camera_on
python ${cur_path}/centralized_case.py    --num_agents ${num_agents} --robot_type ${robot_type}


