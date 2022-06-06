#!/bin/bash

setup_path="$(dirname ${BASH_SOURCE[0]})"/../devel/setup.bash
# for xtark_robot  (use python3 in ros-melodic)
source ${setup_path} --extend
source ~/cv_bridge_ws/install/setup.bash

export ROS_MASTER_URI=http://172.16.0.62:11311
mac2id_path=$(dirname ${BASH_SOURCE[0]})/mac2id.py
export CAR_ID=$(python ${mac2id_path})