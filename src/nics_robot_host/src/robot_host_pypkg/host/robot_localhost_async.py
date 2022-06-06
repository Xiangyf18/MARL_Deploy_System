#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import copy
import threading

from cv_bridge import CvBridge, CvBridgeError
from nics_robot_host.srv import *
from nics_robot_host.msg import *
import time
import numpy as np

from robot_host_pypkg.robot_localhost import add_config,RobotHost as ParentHost

class RobotHost(ParentHost):
    def __init__(self, args):
        super.__init__(args)

    def main_task(self, args):
        self.basic_init()

        # for distribution deployment
        self.share_pub_ = rospy.Publisher("share_info", share_info, queue_size=1)
        self.step_num = -1
        self.latest_share_data=None
        # pub own share_info
        self.share_init_flag=False
        self.pub_share_info_thread=threading.Thread(target=self.pub_share_info,daemon=True)
        self.pub_share_info_thread.start()
        # build observation service
        self.infer_server = None
        
        self.host_init_steps(args)

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

    def store_data(self, msg, args):
        v_id = args[0]
        data_name = args[1]

        if data_name=='camera/depth/image_raw' :
            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'passthrough')
            self.ros_data_interface[v_id][data_name] = copy.deepcopy(cv_image)

        elif data_name=='usb_cam/image_raw' \
            or data_name=='camera/image_raw' :
            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
            self.ros_data_interface[v_id][data_name] = copy.deepcopy(cv_image)

        elif data_name=='pose' or  data_name=='twist' :
            self.ros_data_interface[v_id][data_name] =[msg.twist.linear.x,
                                                        msg.twist.linear.y,msg.twist.angular.z]
        elif data_name=='scan':
            ranges=msg.ranges
            rotated_ranges=np.array([0.0 for i in range(720)])
            for idx, data in enumerate(ranges):
                rotated_ranges[(idx+360) % 720] = data
            self.ros_data_interface[v_id][data_name] =rotated_ranges

        elif data_name=='share_info':
            old_data=self.ros_data_interface[v_id][data_name]
            if old_data==True or old_data.step<msg.step or \
            (old_data.step==msg.step and old_data.data.step<msg.data.step):
                self.ros_data_interface[v_id][data_name] = copy.deepcopy(msg)

        else:
            self.ros_data_interface[v_id][data_name] = copy.deepcopy(msg)

    def pub_share_info(self):
        while True:
            # pub my own share_info
            if self.step_num<0 :
                self.share_pub_.publish(share_info())
            else:
                self.share_pub_.publish(self.GetShareInfo(self.latest_share_data))
            rospy.sleep(0.2)

    def GetShareInfo(self, DATA=None):
        share = share_info()
        share.car_id = self.car_id
        share.step = self.step_num

        data = share_data()
        if DATA == None:
            data.step = -1
            #data.DATA=to_bytes( pickle.dumps(coordinate))
        else:
            data.step = self.step_num
            # data.DATA=to_bytes(pickle.dumps(DATA))
            data.DATA = DATA

        coordinate=self.ros_data_interface[self.car_id]['pose']
        data.coordinate = coordinate
        share.data = data
        return share

    def sync_reset(self):
        # block until 'reset' sync, and not sync 'step'
        while True:
            sync_flag = True
            for v_id, inter in self.ros_data_interface.items():
                if v_id == self.car_id:
                    continue
                step = inter['share_info'].step
                if step < self.step_num:
                    # print(f"{v_id} 's step_num too small ",step,self.step_num)
                    sync_flag = False
                    break
            if sync_flag:
                break
            time.sleep(0.04)

    def step(self, action):
        #convert action and  sync step, 
        self.env.convert_single_action(action,self.car_id)
        self.step_num += 1
        # self.sync_step()
        
        # get obs 
        obs=self.env.convert_single_observation(self.car_id) 
        if  self.lidar_on==False and self.camera_on==False and self.depth_camera_on==False:  
            obs = self.get_others_pose(obs)
        return obs

    def reset(self):
        # wait env init, wait for host reset, sync step, and get obs 
        while self.env==None:
            time.sleep(0.1)
        self.env.reset()
        self.step_num += 1
        self.sync_reset()

        # get obs 
        obs=self.env.convert_single_observation(self.car_id) 
        if  self.lidar_on==False and self.camera_on==False and self.depth_camera_on==False:  
            obs = self.get_others_pose(obs)
        return obs

    def share(self, data:str):
        self.latest_share_data=data
        share_data_list = []
        while True:
            init_share_flag=True
            for v_id, inter in self.ros_data_interface.items():
                if v_id == self.car_id:
                    continue
                #DATA =  pickle.loads(from_bytes( inter['share_info'].data.DATA) )
                DATA = inter['share_info'].data.DATA
                if DATA==None or DATA==str() or DATA=="": # for first share
                    init_share_flag=False
                    break
            if init_share_flag==True:
                break
            time.sleep(0.05)

        for v_id, inter in self.ros_data_interface.items():
            if v_id == self.car_id:
                continue
            DATA = inter['share_info'].data.DATA
            share_data_list.append(DATA)
        return share_data_list
