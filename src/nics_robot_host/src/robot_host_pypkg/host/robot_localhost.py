#!/usr/bin/python3
# -*- coding:utf-8 -*-
import math
import os

#TODO: may change a better way to set a ros namespace in python script
try :
    NAMESPACE_LOCAL_HOST=os.environ["ROS_NAMESPACE"]
except:
    NAMESPACE_LOCAL_HOST=""
if NAMESPACE_LOCAL_HOST==None or NAMESPACE_LOCAL_HOST==" ":
    NAMESPACE_LOCAL_HOST=""
os.environ["ROS_NAMESPACE"] =NAMESPACE_LOCAL_HOST+"/"+os.getenv('CAR_ID')  
import rospy
import rostopic
import rosnode
from geometry_msgs.msg import PoseStamped, TwistStamped
import copy
import threading

from cv_bridge import CvBridge, CvBridgeError
from nics_robot_host.srv import *
from nics_robot_host.msg import *
from rospy.core import rospyinfo
import time
import numpy as np


from robot_host_pypkg.env.environment import MultiRobotEnv
from robot_host_pypkg.env.basic_observation import Observation

class RobotHost(object):
    def __init__(self, args):
        self.car_id: str = os.getenv('CAR_ID')
        self.host_close = False
        self.env=None 
        self._thread_host = threading.Thread(
            target=self.main_task, args=(args,))
        self._thread_host.setDaemon(True)
        self._thread_host.start()

    def main_task(self, args):
        self.basic_init(args)

        # for distribution deployment
        self.share_pub_ = rospy.Publisher("share_info", share_info, queue_size=1)
        self.step_num = -1
        self.last_share_DATA=" "
        # build observation service
        self.infer_server = None
    
        self.host_init_steps(args)

    def basic_init(self,args):
        # get args
        self.agent_num = args.num_agents
        self.lidar_on=args.lidar_on
        self.camera_on=args.camera_on
        self.depth_camera_on=args.depth_camera_on

        # init host node
        rospy.init_node("robot_host", disable_signals=True)
        self.bridge_ = CvBridge()
        self.rate=rospy.Rate(10)

    def host_init_steps(self,args):

        # check the number of agent localhost
        self.wait_for_host_nodes()

        # waiting for all topic
        self.add_all_rostopic()

        # check for all topic
        self.wait_for_all_rostopic()

        # sub all topics
        self.sub_all_rostopic()

        # check for my own  client control service
        self.init_client_service()

        # ensure all msg data init
        self.init_msg_data()

        self.state_flag = 'reset'
        self.env:MultiRobotEnv=MultiRobotEnv(args,self.ros_data_interface)
        while True:
            if self.state_flag == 'reset':
                self.init_for_reset_state()

            if self.state_flag == 'wait for start':
                self.init_for_start_state()

            if self.state_flag == 'start':
                time.sleep(0.5)  # 可能还会reset

    def wait_for_host_nodes(self):
        # check the number of agent localhost
        All_ready = False
        while not All_ready:
            node_name_list: list[str] = rosnode.get_node_names()
            self.vehicle_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_host'):
                    # assume all robot_host note named as '/XXXX/vehicle_id/robot_host'
                    self.vehicle_id_list.append(node_name.split('/')[-2])
            print('wait for all robot_hosts , now: %s' %
                  self.vehicle_id_list)
            if len(self.vehicle_id_list) == self.agent_num:
                All_ready = True
                break
            rospy.sleep(0.5)
        rospy.loginfo('All robot_hosts ready ')

    def add_all_rostopic(self):
        self.ros_data_interface:dict = {}
        for vehicle_id in self.vehicle_id_list:
            ros_data:dict = {}
            if vehicle_id != self.car_id:
                ros_data['share_info'] = False
            else:
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
            # pub my own share_info
            self.share_pub_.publish(self.GetShareInfo())

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

    def sub_all_rostopic(self):
        self.ros_data_interface_sub = []
        # subscribe all ros data interface
        for vehicle_id in self.vehicle_id_list:
            for data_name in self.ros_data_interface[vehicle_id].keys():
                # handle = lambda msg: store_data_2(msg, self.ros_data_interface, vehicle_id, data_name)
                topic_name = '/'+vehicle_id+'/'+data_name
                data_class = rostopic.get_topic_class(topic_name)[0]
                sub = rospy.Subscriber(name=topic_name,
                                        data_class=data_class,
                                        callback=self.store_data,
                                        callback_args=(vehicle_id, data_name),
                                        queue_size=1)
                self.ros_data_interface_sub.append(sub)
        for sub in self.ros_data_interface_sub:
            print('sub_name', sub.name)

    def init_client_service(self):
        # check for my own  client control service
        client_ctrl_name = '/'+self.car_id+'/client_control'
        rospy.wait_for_service(client_ctrl_name)
        self.client_ctrl_srv = rospy.ServiceProxy(client_ctrl_name, sup)

        self.ros_spin_thread = threading.Thread(target=rospy.spin)
        self.ros_spin_thread.setDaemon(True)
        self.ros_spin_thread.start()

    def init_msg_data(self):
        # ensure all msg data init
        while True:
            topics_ready = True
            # pub my own share_info
            share_ = self.GetShareInfo()
            self.share_pub_.publish(share_)

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

    def init_for_reset_state(self):
        # 提示env层：core 进程已经结束，已进入reset状态，对方线程可修改agent.pos
        self.env.real_world_reset = False
        while True:
            if self.env.virtual_world_init == True:
                break
            time.sleep(0.01)

        self.state_flag = 'wait for start'
        self.env.real_world_connection = False
        self.env.virtual_world_init = False
        self.step_num = 0
        rospy.loginfo('Muti-Robot Platform reset!')

    def init_for_start_state(self):
        # add ros service : infer
        if self.infer_server == None:
            vehicle_id = self.car_id
            inference_service_name = '/' + vehicle_id + '/inference'
            inference_service = rospy.Service(name=inference_service_name,
                                                service_class=infer,
                                                handler=(lambda a: lambda msg: self.inference(msg, a))(vehicle_id,))
            self.infer_server = inference_service

        self.core_thread = threading.Thread(target=self.core_function)
        self.core_thread.setDaemon(True)
        self.core_thread.start()

        sup_arg = supRequest()
        sup_arg.start = True
        sup_arg.movable = True
        sup_arg.collision = False
        self.client_ctrl_srv(sup_arg)

        self.state_flag = 'start'
        rospy.loginfo('Muti-Robot Platform start!')

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

        else:
            self.ros_data_interface[v_id][data_name] = copy.deepcopy(msg)

    def inference(self, req, vehicle_id):
        if  self.env.step_on_dict[vehicle_id]==True:
            action = self.env.real_actions_dict[vehicle_id]
        else:
            action=np.array([0, 0, 0])
        return inferResponse(action)

    def core_function(self):
        while self.env.real_world_reset == False and self.host_close == False:  # 可中断
            # send  information ,also as  the connection-check
            sup_arg = supRequest()
            sup_arg.start = True
            sup_arg.movable = True
            sup_arg.collision = False
            try:
                self.client_ctrl_srv(sup_arg)
            except rospy.ServiceException as exc:
                rospy.loginfo("Robot client %s Service Close: %s" %
                                (self.car_id, str(exc)))

            self.env.real_world_connection = True  # 在此表示host初始化成功
            self.rate.sleep()
        # 退出并重启
        self.state_flag = 'reset'

    def GetShareInfo(self, DATA=None):
        share = share_info()
        share.car_id = self.car_id
        share.step = self.step_num

        data = share_data()
        if DATA == None:
            data.step = -1
            data.DATA=self.last_share_DATA
            #data.DATA=to_bytes( pickle.dumps(coordinate))
        else:
            data.step = self.step_num
            # data.DATA=to_bytes(pickle.dumps(DATA))
            data.DATA = DATA
            self.last_share_DATA=DATA

        coordinate=self.ros_data_interface[self.car_id]['pose']
        data.coordinate = coordinate
        share.data = data
        return share

    def sync_step(self):
        # block until step sync
        while True:
            sync_flag = True
            self.share_pub_.publish(self.GetShareInfo())
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

    def get_others_pose(self,own_obs):
        # 兼容之前版本的obs格式
        # 仅使用pose的情况下，返回所有agent的obs
        obs=[own_obs]
        for robot_id in self.env.robot_id_list:
                if robot_id == self.car_id:
                    continue
                inter=self.ros_data_interface[robot_id]
                coordinate = inter['share_info'].data.coordinate
                obs.append(Observation(float(coordinate[0]), 
                                        float(coordinate[1]), 
                                        float(coordinate[2])))
        return obs

    def step(self, action):
        #convert action and  sync step, 
        self.env.convert_single_action(action,self.car_id)
        self.step_num += 1
        self.sync_step()
        
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
        self.sync_step()

        # get obs 
        obs=self.env.convert_single_observation(self.car_id) 
        if  self.lidar_on==False and self.camera_on==False and self.depth_camera_on==False:  
            obs = self.get_others_pose(obs)
        return obs

    def share(self, data:str):
        share_ = self.GetShareInfo(DATA=data)
        share_data_list = []
        while True:
            sync_flag = True
            self.share_pub_.publish(share_)
            for v_id, inter in self.ros_data_interface.items():
                if v_id == self.car_id:
                    continue
                data = inter['share_info'].data
                if data.step < self.step_num and self.step_num>=inter['share_info'].step:
                    sync_flag = False
                    break
            if sync_flag:
                break
            time.sleep(0.05)

        for v_id, inter in self.ros_data_interface.items():
            if v_id == self.car_id:
                continue
            DATA = inter['share_info'].data.DATA
            share_data_list.append(DATA)
        return share_data_list

    def close(self):
        self.host_close = True


def add_config(parser):
    # not use now  please add in (for example   "gridworld/base/config.py")

    # 注意防止与上层的命名冲突
    # for deployment
    # parser.add_argument('--num_agents', type=int,
    #                    default=1, help="number of players")
    # 根据小车类型选择 Ackerman  Mecanum
    parser.add_argument('--robot_type', type=str, default='Mecanum')
    parser.add_argument('--lidar_on', default=False, action='store_true')
    parser.add_argument('--camera_on', default=False, action='store_true')
    parser.add_argument('--depth_camera_on', default=False, action='store_true')
    return parser
