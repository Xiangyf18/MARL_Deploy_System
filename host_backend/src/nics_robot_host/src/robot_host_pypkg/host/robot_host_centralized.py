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

from robot_host_pypkg.robot_localhost import add_config,RobotHost as ParentHost

class RobotHost(ParentHost):
    def __init__(self, args):
        super.__init__(args)


    def main_task(self, args):
        self.basic_init(args)

        # build multi-thread for action
        self.action_thread_list=[]
        # build observation service
        self.infer_server_list = []
    
        self.host_init_steps(args)

    def wait_for_host_nodes(self):
        # check the number of agent localhost
        All_ready = False
        while not All_ready:
            node_name_list: list[str] = rosnode.get_node_names()
            self.vehicle_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_client'):
                    # assume all robot_client note named as '/XXXX/vehicle_id/robot_client'
                    self.vehicle_id_list.append(node_name.split('/')[-2])
            print('wait for all robot_clients , now: %s' %
                  self.vehicle_id_list)
            if len(self.vehicle_id_list) == self.agent_num:
                All_ready = True
                break
            rospy.sleep(0.5)
        rospy.loginfo('All robot_clients ready ')

    def add_all_rostopic(self):
        self.ros_data_interface:dict = {}
        for vehicle_id in self.vehicle_id_list:
            ros_data:dict = {}
            sensors = ['pose','twist']
            if self.lidar_on == True:
                sensors.append('scan')
            if self.camera_on == True:
                sensors.append('camera/image_raw')
            if self.depth_camera_on == True:
                sensors.append('camera/depth/image_raw')
                sensors.append('usb_cam/image_raw')
            for data_name in sensors:
                ros_data[data_name] = False
            self.ros_data_interface[vehicle_id] = ros_data

    def wait_for_all_rostopic(self):
        all_data_interface_ready = False
        while not all_data_interface_ready:
            all_data_interface_ready = True
            for v_id, inter in self.ros_data_interface.items():
                for data_name, state in inter.items():
                    if state is False:
                        print('/'+v_id+'/' + data_name + ' is not found')
                        all_data_interface_ready = False

            topic_list = rospy.get_published_topics()
            for topic in topic_list:
                topic_name: str = topic[0]
                topic_name_split = topic_name.split('/')
                v_id = topic_name_split[1]

                if v_id in self.ros_data_interface.keys():
                    data_name = topic_name_split[2]
                    if data_name in self.ros_data_interface[v_id]:
                        self.ros_data_interface[v_id][data_name] = True
                    if (self.camera_on == True or self.depth_camera_on == True ) \
                        and len(topic_name_split) == 4:
                        data_name = topic_name_split[2]+'/'+topic_name_split[3]
                        if data_name in self.ros_data_interface[v_id]:
                            self.ros_data_interface[v_id][data_name] = True
                    if self.depth_camera_on == True and len(topic_name_split) == 5:
                        data_name = topic_name_split[2]+'/'+topic_name_split[3]+'/'+topic_name_split[4]
                        if data_name in self.ros_data_interface[v_id]:
                            self.ros_data_interface[v_id][data_name] = True

            rospy.sleep(1.0)

    def init_client_service(self):
        # check for all client control services
        self.client_ctrl_srv = []
        for vehicle_id in self.vehicle_id_list:
            client_ctrl_name = '/'+vehicle_id+'/client_control'
            rospy.wait_for_service(client_ctrl_name)
            self.client_ctrl_srv.append(
                rospy.ServiceProxy(client_ctrl_name, sup))

        self.ros_spin_thread = threading.Thread(target=rospy.spin)
        self.ros_spin_thread.setDaemon(True)
        self.ros_spin_thread.start()

    def init_msg_data(self):
        # ensure all msg data init
        while True:
            topics_ready = True
            for v_id in self.vehicle_id_list:
                for data_name in self.ros_data_interface[v_id].keys():
                    if type(self.ros_data_interface[v_id][data_name]) != type(np.array([])) and \
                    self.ros_data_interface[v_id][data_name] == True:
                        topics_ready = False
                        print(f'{v_id}/{data_name} topic data not init')
                        break
            if topics_ready == True:
                break
            rospy.sleep(0.5)
        rospy.loginfo('All rostopic ready ')

    def init_for_start_state(self):
        # add ros service : infer
        if len(self.infer_server_list) == 0:
            for vehicle_id in self.vehicle_id_list:
                inference_service_name = '/' + vehicle_id + '/inference'
                inference_service = rospy.Service(name=inference_service_name,
                                                    service_class=infer,
                                                    handler=(lambda a: lambda msg: self.inference(msg, a))(vehicle_id,))
                self.infer_server_list.append(inference_service)

        self.core_thread = threading.Thread(target=self.core_function)
        self.core_thread.setDaemon(True)
        self.core_thread.start()

        for idx in range(len(self.vehicle_id_list)):
            sup_arg = supRequest()
            sup_arg.start = True
            sup_arg.movable = True
            sup_arg.collision = False
            self.client_ctrl_srv[idx](sup_arg)

        self.state_flag = 'start'
        rospy.loginfo('Muti-Robot Platform start!')

    def core_function(self):
        while self.env.real_world_reset == False and self.host_close == False:  # 可中断
            # send  information ,also as  the connection-check
            for v_idx in range(self.agent_num):
                robot_id = self.vehicle_id_list[v_idx]
                sup_arg = supRequest()
                sup_arg.start = True
                sup_arg.movable = True
                sup_arg.collision = False
                try:
                    self.client_ctrl_srv[v_idx](sup_arg)
                except rospy.ServiceException as exc:
                    rospy.loginfo("Robot client %s Service Close: %s" %
                                    (robot_id, str(exc)))

            self.env.real_world_connection = True  # 在此表示host初始化成功
            self.rate.sleep()
        # 退出并重启
        self.state_flag = 'reset'

    def step(self, action):
        # check action num
        assert len(action)==len(self.vehicle_id_list)

        # multi action threads , and wait to sync
        self.action_thread_list:List[threading.Thread]=[]
        for idx,robot_id in enumerate( self.vehicle_id_list):
            action_thread=threading.Thread(target=self.env.convert_single_action(action[idx],robot_id),daemon=True)
            action_thread.start()
            self.action_thread_list.append(action_thread)
        for t in self.action_thread_list:
            t.join()

        obs_n=[]
        for robot_id in self.vehicle_id_list:
            obs_n.append(self.env.convert_single_observation(robot_id))
        return obs_n

    def reset(self):
        # wait env init, wait for host reset, and get obs 
        while self.env==None:
            time.sleep(0.1)
        self.env.reset()

        # get obs 
        obs_n=[]
        for robot_id in self.vehicle_id_list:
            obs_n.append(self.env.convert_single_observation(robot_id))
        return obs_n
