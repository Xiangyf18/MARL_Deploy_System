#!/bin/bash

# This is main shell script for centralized deployment on multi-robot
#assure all files have existed

#ip config
user="xtark" 
key_="xtark"

num_agents=$2
lidar_on=0
camera_on=0
depth_camera_on=0
robot_type=$1



if [ ${robot_type} == "Mecanum" ]
then
    #MKN_1
    # ip_array[0]="172.16.0.110"
    ip_array[0]="172.16.1.62"
    #MKN_2
    ip_array[1]="172.16.0.114"
    # ip_array[1]="172.16.1.61"
    #MKN_4
    ip_array[2]="172.16.1.61"
    #MKN_5
    ip_array[3]="172.16.1.62"
    #MKN_3
    ip_array[4]="192.168.50.139"


    #TODO

    #input num_agents

    if [ ${num_agents} -gt 4 ]
    then
        echo "agent_num should not greater than 4!"
        exit 1
    fi

elif [ ${robot_type} == "Ackerman" ]
then
    #AKM_4
    ip_array[0]="192.168.50.208" 
    #AKM_1
    ip_array[1]="192.168.50.91" 
    #AKM_2
    ip_array[2]="192.168.50.203"
    #AKM_3
    ip_array[3]="192.168.50.207"
    #AKM_5 
    ip_array[4]="192.168.50.64"

    #input num_agents

    if [ ${num_agents} -gt 5 ]
    then
        echo "agent_num should not greater than 5!"
        exit 1
    fi

fi




echo "[INFO]: All running proceedings for sensor and driver on Robots will be killed before the deployment system starts! "

#check ‘lidar’  ‘depth_camera’ 
for i in "$@"; do  
    if [ $i == "lidar" ]
    then
        echo "[INFO]: open Lidar"
        lidar_on=1 
    fi
    if [ $i == "camera" ]
    then
        echo "[INFO]: open Camera "
        camera_on=1 
    fi
    if [ $i == "depth_camera" ]
    then
        echo "[INFO]: open Depth Camera "
        depth_camera_on=1 
    fi
done



#start roscore 
export ROS_MASTER_URI=http://172.16.0.62:11311
export ROS_IP=172.16.0.62
{ 
    ps -ef | grep ros | awk '{print $2}' | xargs kill -9 > /dev/null 2>&1 
    roscore > /dev/null 2>&1 
} & 

echo '[INFO]: Sleep 2 seconds to wait roscore starts'
sleep 2

#start optitrack
source /home/ubuntu/project/NICS_MultiRobot_Platform/devel/setup.bash
{
    ps -ef | grep vrpn_client | awk '{print $2}' | xargs kill -9 > /dev/null 2>&1 
    ps -ef | grep vrpn_location | awk '{print $2}' | xargs kill -9 > /dev/null 2>&1
    roslaunch vrpn_client_ros vrpn_location_pose.launch  > /dev/null 2>&1 
} &

#echo 'Sleep 2 seconds to wait OptiTrack starts'  不需要了，在RL中进行
#sleep 2


for index in $(seq 0 $(expr $num_agents - 1) )
do
{
ip=${ip_array[index]}
#ssh xtark@ip > /dev/null 2>&1 << eeooff
sshpass -p ${key_} ssh -tt ${user}@${ip} > /dev/null  2>&1 << eeooff 
cd /home/xtark/sim2real/sim2real/hosts/host_backend
chmod +x scripts/setup_client.sh
ps -ef | grep NICS_MultiRobot_Platform | awk '{print \$2}' | xargs kill -9
ps -ef | grep sim2real | awk '{print \$2}' | xargs kill -9
ps -ef | grep robot_client | awk '{print \$2}' | xargs kill -9
ps -ef | grep xtark_driver | awk '{print \$2}' | xargs kill -9
ps -ef | grep setup_client | awk '{print \$2}' | xargs kill -9
ps -ef | grep camera | awk '{print \$2}' | xargs kill -9
ps -ef | grep laser | awk '{print \$2}' | xargs kill -9
nohup ./scripts/setup_client.sh $lidar_on $camera_on $depth_camera_on &
exit
eeooff
} &

done

setup_path="$(dirname ${BASH_SOURCE[0]})"/../devel/setup.bash
source ${setup_path}
