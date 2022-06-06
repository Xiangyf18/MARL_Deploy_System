#!/usr/bin/python3
# -*- coding:utf-8 -*-
import os
os.environ["ROS_NAMESPACE"] ="" 

import rospy
import rosnode
import copy
import threading

from cv_bridge import CvBridge, CvBridgeError
from nics_robot_host.srv import *
from nics_robot_host.msg import *
import time
import numpy as np
from typing import List

from robot_host_pypkg.robot_host_centralized import add_config,RobotHost as ParentHost

class RobotHost(ParentHost):
    def __init__(self, args):
        super.__init__(args)

    def basic_init(self,args):
        # get args
        self.agent_num = args.num_agents
        self.lidar_on=args.lidar_on
        self.camera_on=args.camera_on
        self.depth_camera_on=args.depth_camera_on

        # init host node
        rospy.init_node("visual_host", disable_signals=True)
        self.bridge_ = CvBridge()
        self.rate=rospy.Rate(10)

    def init_for_start_state(self):
        self.core_thread = threading.Thread(target=self.core_function)
        self.core_thread.setDaemon(True)
        self.core_thread.start()

        self.state_flag = 'start'
        rospy.loginfo('Muti-Robot Platform start!')

    def core_function(self):
        while self.env.real_world_reset == False and self.host_close == False:  # 可中断
            self.env.real_world_connection = True  # 在此表示host初始化成功
            self.rate.sleep()
        # 退出并重启
        self.state_flag = 'reset'

    def step(self, action):
        obs_n=[]
        for robot_id in self.vehicle_id_list:
            obs_n.append(self.env.convert_single_observation(robot_id))
        return obs_n
