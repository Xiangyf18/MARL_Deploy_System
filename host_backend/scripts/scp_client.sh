#!/bin/bash
cur_path=$(dirname ${BASH_SOURCE[0]})
# aim to quickly update the files on robots



#update clients
sshpass -p xtark ssh -tt xtark@$1 > /dev/null  2>&1 << eeooff 
cd /home/xtark/
rm -rf NICS_MultiRobot_Platform
mkdir -p NICS_MultiRobot_Platform/src/nics_robot_client
mkdir -p NICS_MultiRobot_Platform/scripts
exit
eeooff

sshpass -p xtark scp -r ${cur_path}/../src/nics_robot_client/* xtark@$1:/home/xtark/NICS_MultiRobot_Platform/src/nics_robot_client
sshpass -p xtark scp -r ${cur_path}/* xtark@$1:/home/xtark/NICS_MultiRobot_Platform/scripts/
sshpass -p xtark scp -r ${cur_path}/../.catkin_workspace xtark@$1:/home/xtark/NICS_MultiRobot_Platform/

sshpass -p xtark ssh -tt xtark@$1 > /dev/null  2>&1 << eeooff 
cd /home/xtark/NICS_MultiRobot_Platform
catkin_make
exit
eeooff
