#!/usr/bin/python3
# -*- coding:utf-8 -*-
import time
from typing import Any, Dict, List
import gym
from gym import spaces
import numpy as np
import threading

from .basic_action import *
from .basic_observation import *

from .utils import *


class MultiRobotEnv(object):
    def __init__(self, args, ros_data_interface):
        self.agent_num=args.num_agents
        self.ros_data_interface:dict = ros_data_interface
        self.robot_id_list:list =  self.ros_data_interface.keys()

        # args for observation
        self.lidar_on=args.lidar_on
        self.camera_on=args.camera_on
        self.depth_camera_on=args.depth_camera_on

        # args for motion
        self.real_actions_dict=dict(zip(self.robot_id_list,[[0.0,0.0,0.0] for i in range(self.agent_num)]))
        self.step_on_dict=dict(zip(self.robot_id_list,[False for i in range(self.agent_num)])) #signal，告知host可以持续输出动作
        self.speed_coefficient_dict=dict(zip(self.robot_id_list, # get specific speed coefficient
                                            [GetVelocityCoefficient(robot_id) for robot_id in self.robot_id_list]))

        if args.robot_type=='Ackerman':
            self.action_space=spaces.Box(low=-7.0, high=7.0, shape=(2,))
            self.robot_type='Ackerman'
        elif args.robot_type=='Mecanum':
            self.action_space=spaces.Box(low=-7.0, high=7.0, shape=(3,))
            self.robot_type='Mecanum'
        else:
            print(f"[ERROR]: Unknown robot_type [{args.robot_type}] !")   

        # args for  reset 
        self.real_world_connection=False #在real_world reset后触发
        self.virtual_world_init=False #在real_world_reset之前完成
        self.real_world_reset=False  #控制量
        

    def reset(self):
        # trigger for reset 
        if self.real_world_connection == True: # if the system init and is working
            self.real_world_reset = True

        # waiting for robot-host to  start reset 
        while True:  
            if self.real_world_reset == False: # robot-host in reset mode
                break
            time.sleep(0.01)
        # reset  virtual world data
        self.real_world_connection = False
        # ... reset(something)
        self.virtual_world_init = True

        # waiting for robot-host to finish reset
        while True: 
            if self.real_world_connection == True:
                break
            time.sleep(0.1)  

    '''
    def put_into_action(self, action_n):
        if self.local_id==None: # for centralized deployment
            # multi thread to step
        else: # for distributied deployment
            self.convert_single_action(action_n,self.local_id)
    '''

    '''
    def get_obs(self):  
        if self.local_id==None : # for centralized deployment
            obs_n=[]
            for robot_id in self.robot_id_list:    
                obs_n.append(self.convert_single_observation(robot_id))
            return obs_n

        else: # for distributied deployment
            obs=self.convert_single_observation(self.local_id)
            
            if  self.lidar_on==False and self.camera_on==False and self.depth_camera_on==False:  
            # 仅有位置坐标的情况下，兼容之前版本的obs格式
                obs = [obs]
                for robot_id in self.robot_id_list:
                    if robot_id == self.local_id:
                        continue
                    inter=self.ros_data_interface[robot_id]
                    coordinate = inter['share_info'].data.coordinate
                    obs.append(Observation(float(coordinate[0]), 
                                                  float(coordinate[1]), 
                                                  float(coordinate[2])))
            return obs
    '''

    def convert_single_action(self,basic_action,robot_id:str):
        # for single agent, convert basic_action to real_action ([x,y,yaw])
        # 兼容之前版本的‘goal’类型
        if (basic_action.type == "goal" or  basic_action.type =="discrete"):
            if self.robot_type != 'Mecanum':
                print(f'[ERROR] It\'s not allowed to use discrete action for robots with {self.robot_type} steering ')
                return False
            self.step_on_dict[robot_id]=True
            target_action = np.array([0, 0, 0])
            target_pose = None
            n_dir= 10 if basic_action.type =="discrete" else 45

            data_pose = self.ros_data_interface[robot_id]['pose']
            if basic_action.direction >= 0:

                theta_ctrl = basic_action.direction*n_dir
                theta = int(data_pose[2]/np.pi*180)
                target_theta = round(
                    np.mod(theta+theta_ctrl, 360)/n_dir)*n_dir  # 0~36 车朝向限定为n_dir个方位内
            else:
                target_theta = basic_action.yaw
                theta = int(data_pose[2]/np.pi*180)
                theta_ctrl = np.mod(target_theta-theta, 360)  # 不限制在n_dir个方位内

            if basic_action.distance >= 0:
                # 坐标系转换:local->world
                rel_target_theta = theta_ctrl/180*np.pi
                rel_target_x = basic_action.distance*np.cos(rel_target_theta)
                rel_target_y = basic_action.distance*np.sin(rel_target_theta)

                theta = theta/180*np.pi
                target_x, target_y = RotateAxis(
                    rel_target_x, rel_target_y, -theta)
                target_x += data_pose[0]
                target_y += data_pose[1]
            else:
                target_x, target_y = basic_action.position
            target_pose = [target_theta, target_x, target_y]

            # rotate , and then go straight
            while True:
                flag = 0
                # update data
                data_pose = self.ros_data_interface[robot_id]['pose']
                data_velocity = self.ros_data_interface[robot_id]['twist']

                target_theta, target_x, target_y = target_pose
                # GET delta_theta  
                theta = int(data_pose[2]/np.pi*180)
                delta_theta = target_theta-np.mod(theta, 360)
                if abs(delta_theta) > 180:
                    delta_theta = delta_theta+360 if delta_theta <= -180 else delta_theta-360
                if abs(delta_theta) > 5:
                    delta_theta = clip(delta_theta, 
                                        high=90, 
                                        low=-90, 
                                        bound=8,
                                        k=1.0/180*np.pi*1.1)
                    target_action = np.array([0, 0, delta_theta])*self.speed_coefficient_dict[robot_id]
                    #print(f'[{vehicle_id}]  delta_theta: {delta_theta}')
                    self.real_actions_dict[robot_id]=self.real_action_check(target_action,robot_id)
                    continue

                # GET delta_distance  
                # 目标点的坐标系转换：world->local
                related_x = target_x-data_pose[0]
                related_y = target_y-data_pose[1]
                related_theta = data_pose[2]
                delta_rel_x, delta_rel_y = RotateAxis(
                    related_x, related_y, related_theta)
                if abs(delta_rel_x) <= 0.025 and abs(delta_rel_y) <= 0.025:
                    flag += 1
                    self.real_actions_dict[robot_id] = np.array([0, 0, 0])
                    continue

                # GET real-action
                vel_x=data_velocity[0] if not  math.isnan(data_velocity[0]) else 0.0
                vel_y=data_velocity[1] if not  math.isnan(data_velocity[1]) else 0.0
                rel_vel_x,rel_vel_y=RotateAxis(vel_x,vel_y, related_theta)
                delta_rel_x = clip(delta_rel_x,speed=rel_vel_x)
                delta_rel_y = clip(delta_rel_y,speed=rel_vel_y)
                target_action = np.array([
                    delta_rel_x, delta_rel_y, 0])*self.speed_coefficient_dict[robot_id]
                # print(f'[{vehicle_id}]   delta_rel_x: {delta_rel_x} , delta_rel_y: {delta_rel_y}')
                self.real_actions_dict[robot_id]=self.real_action_check(target_action,robot_id)
                
                if flag == 1:
                    self.step_on_dict[robot_id]=False
                    break
                time.sleep(0.01)

        elif basic_action.type == "speed":
            self.step_on_dict[robot_id]=True
            user_ratio=np.array([basic_action.forward,basic_action.left,basic_action.anticlockwise],dtype=np.float32)
            # get  standard velocity for each robot
            yaw_=clip(90.0, 
                    high=90, 
                    low=-90, 
                    bound=8,
                    k=1.0/180*np.pi*1.1)
            x_=clip(0.3,speed=0.0)
            y_=x_
            target_action=np.array([x_,y_,yaw_])*self.speed_coefficient_dict[robot_id]*user_ratio
            self.real_actions_dict[robot_id]=self.real_action_check(target_action,robot_id)        
        else:
            print(f"[ERROR] Unknown action_type [{basic_action.type}]") 

    def real_action_check(self,real_action,robot_id):
        real_action = np.array(real_action, dtype=np.float32)
        if not self.action_space.contains(real_action):
            print(
                f"[ERROR]: Agent [{robot_id}]'s action {real_action} is not within the safe speed range ! ")
            real_action = np.array([0.0,0.0,0.0],dtype=np.float32)
        return real_action

    def convert_single_observation(self,robot_id:str):
        # for single agent, convert real data to basic_observation
        inter=self.ros_data_interface[robot_id]
        pose=inter['pose']
        if self.depth_camera_on == True:
            return ObservationWithRGBD(x=float(pose[0]), 
                                        y=float(pose[1]), 
                                        yaw=float(pose[2]),
                                        depth=inter['camera/depth/image_raw'],
                                        img=inter['usb_cam/image_raw'])
        elif self.lidar_on==True:
            #TODO:
            return None
        else: # for no sensor situation
            return Observation(x=float(pose[0]), 
                                y=float(pose[1]), 
                                yaw=float(pose[2]))


       





