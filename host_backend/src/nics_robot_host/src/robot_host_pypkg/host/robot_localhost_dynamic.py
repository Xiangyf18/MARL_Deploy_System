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
import copy
import threading

from cv_bridge import CvBridge, CvBridgeError
from nics_robot_host.srv import *
from nics_robot_host.msg import *
from rospy.core import rospyinfo
import time
import numpy as np

from MultiRobotEnv.world import World
from MultiRobotEnv.environment import MultiVehicleEnv
import MultiRobotEnv.scenarios as scenarios

from sim2real.hosts.mecanum import Action as MecAction
from sim2real.hosts.mecanum import Observation as MecObs

from robot_host_pypkg.host_utils import *
import pickle

#TODO:
# 未完成状态，因为没需求了，所以废弃
# 作用：ctrl+C 上下线之后，动态检测robot


# 思路
# 监听线程修改share_info_dict (with lock?),subscribe/unsubscribe，
# 保证agent num输入为1，这样只会init自己，其他代码不用改
# share 中反复等待dict数据更新即可   ————因为另一个线程实时更新dict(包含key增减)

#监听rosnode list 会有僵死的node，因为没有考虑过tcp连接断开的情况，远程的rosnode与本机上的rosnode不同


class RobotHost(object):
    def __init__(self, args):
        self.car_id: str = os.getenv('CAR_ID')
        # load scenario from script
        if not args.deploy_scenario_name:
            args.deploy_scenario_name = "deploy_scenario"
        scenario = scenarios.load(args.deploy_scenario_name + ".py").Scenario()
        # create world
        world: World = scenario.make_world(args)
        # create multiagent environment
        # env = MultiVehicleEnv(world, scenario.reset_world, scenario.reward, scenario.observation,scenario.info)
        self.env = MultiVehicleEnv(
            world, scenario.reset_world, None, scenario.observation, None)

        self.host_close = False
        self._thread_host = threading.Thread(
            target=self.main_task, args=(args,))
        self._thread_host.setDaemon(True)
        self._thread_host.start()

    def main_task(self, args):

        # init host node
        rospy.init_node("robot_host", disable_signals=True)

        # to dynamically detect robot :online/offline
        self.share_info_dict={}
        self.share_info_dict_lock=threading.Lock()
        self.dynamic_detection_thread=threading.Thread(target=self.Dynamic_Detection,daemon=True)
        self.dynamic_detection_thread.start()
        

        # fpr distribution deployment
        self.share_pub_ = rospy.Publisher("share_info", share_info, queue_size=1)
        self.step_num = -1
        # get agent number from env
        self.agent_num = len(self.env.world.vehicle_list)
        if self.agent_num !=1:
            print("[ERROR]:当前版本需要设置agent_num=1")
            return
        self.bridge_ = CvBridge()

        self.core_fps = 10

        # check the number of agent localhost
        self.vehicle_id_list = [str(self.car_id)]
        '''
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
        '''

        # build observation service
        self.infer_server = None
        self.infer_step_flag = False

        # update the agent data_interface
        new_interface = {}
        for vehicle_idx in range(self.agent_num):
            vehicle_id = self.vehicle_id_list[vehicle_idx]
            vehicle = self.env.world.vehicle_list[vehicle_idx]
            old_id = vehicle.vehicle_id
            vehicle.vehicle_id = vehicle_id
            new_interface[vehicle_id] = self.env.world.data_interface.pop(
                old_id)
        self.env.world.data_interface = new_interface

        # waiting for all topic
        self.ros_data_interface = {}
        for vehicle_id in self.vehicle_id_list:
            ros_data = {}
            if vehicle_id != self.car_id:
                ros_data['share_info'] = False
            else:
                sensors = ['pose']
                if self.env.world.lidar_on == True:
                    sensors.append('scan')
                if self.env.world.camera_on == True:
                    sensors.append('camera/image_raw')
                for data_name in sensors:
                    ros_data[data_name] = False
            self.ros_data_interface[vehicle_id] = ros_data
            # print(ros_data)

        # check for all topic
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
                    if self.env.world.camera_on == True and len(topic_name_split) == 4:
                        data_name = topic_name_split[2]+'/'+topic_name_split[3]
                        if data_name in self.ros_data_interface[v_id]:
                            self.ros_data_interface[v_id][data_name] = True
            rospy.sleep(1.0)

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
                                       callback_args=(vehicle_id, data_name))
                self.ros_data_interface_sub.append(sub)
        for sub in self.ros_data_interface_sub:
            print('sub_name', sub.name)

        # check for my own  client control service
        client_ctrl_name = '/'+self.car_id+'/client_control'
        rospy.wait_for_service(client_ctrl_name)
        self.client_ctrl_srv = rospy.ServiceProxy(client_ctrl_name, sup)

        self.ros_spin_thread = threading.Thread(target=rospy.spin)
        self.ros_spin_thread.setDaemon(True)
        self.ros_spin_thread.start()

        # ensure all msg data init
        while True:
            topics_ready = True
            # pub my own share_info
            share_ = self.GetShareInfo()
            self.share_pub_.publish(share_)

            for v_id in self.vehicle_id_list:
                for data_name in self.ros_data_interface[v_id].keys():
                    if self.ros_data_interface[v_id][data_name] == True:
                        topics_ready = False
                        print(f'{v_id}/{data_name} topic data not init')
                        break
            if topics_ready == True:
                break
            rospy.sleep(0.5)
        rospy.loginfo('All rostopic ready ')

        self.state_flag = 'reset'
        while True:
            # cmd = input('state is %s, waiting for cmd '%self.state_flag)
            if self.state_flag == 'reset':
                # 提示env层：core 进程已经结束，已进入reset状态，对方线程可修改agent.pos
                self.env.world.real_world_reset = False
                while True:
                    if self.env.world.virtual_world_init == True:
                        break
                    time.sleep(0.01)

                self.state_flag = 'wait for pos'
                self.env.world.real_world_connection = False
                self.env.world.virtual_world_init = False
                self.step_num = 0
                rospy.loginfo('Muti-Robot Platform reset!')

            # Real Env
            if self.state_flag == 'wait for pos':
                if self.env.world.init_pose_check == True:
                    # TODO： now work in this  version
                    # self.env.reset()
                    while True:
                        result = self.waiting_for_vehicle()
                        if result is True:
                            # rospy.loginfo('All Robots are ready')
                            break
                        else:
                            print(result)
                            rospy.sleep(0.5)
                    self.state_flag = 'wait for start'
                    rospy.loginfo('intended Robot pose init!')

                else:
                    # self.env.reset()
                    for agent in self.env.world.vehicle_list:
                        if self.car_id!=agent.vehicle_id:
                            continue
                        ros_data_interface = self.ros_data_interface[agent.vehicle_id]
                        agent.state.coordinate[0] = ros_data_interface['pose'].twist.linear.x
                        agent.state.coordinate[1] = ros_data_interface['pose'].twist.linear.y
                        agent.state.theta = ros_data_interface['pose'].twist.angular.z
                    self.state_flag = 'wait for start'
                    rospy.loginfo('random Robot pose init!')

            if self.state_flag == 'wait for start':
                # 增加infer服务
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

            if self.state_flag == 'start':
                time.sleep(0.5)  # 可能还会reset

    def waiting_for_vehicle(self):
        def near_enough(x, y, yaw, x_t, y_t, yaw_t):
            # not near_enough distance of agent and the reset agent larger than 0.01m
            if ((x-x_t)**2 + (y-y_t)**2)**0.5 > 0.05:
                return False
            # if yaw and yaw_t distance is larger than 5 degree
            sin_con = math.sin(abs(yaw - yaw_t)) < math.sin(10/180*3.1415)
            cos_con = math.cos(abs(yaw - yaw_t)) > math.cos(10/180*3.1415)
            if not(sin_con and cos_con):
                return False
            return True

        for agent in self.env.world.vehicle_list:
            ros_data_interface = self.ros_data_interface[agent.vehicle_id]
            x_t = agent.state.coordinate[0]
            y_t = agent.state.coordinate[1]
            yaw_t = agent.state.theta
            x = ros_data_interface['pose'].twist.linear.x
            y = ros_data_interface['pose'].twist.linear.y
            yaw = ros_data_interface['pose'].twist.angular.z
            if not near_enough(x, y, yaw, x_t, y_t, yaw_t):
                info_str = "%s pos is (%f, %f, %f) but (%f, %f, %f) is required" % (
                    agent.vehicle_id, x, y, yaw, x_t, y_t, yaw_t)
                return info_str
        return True

    def store_data(self, msg, args):
        v_id = args[0]
        data_name = args[1]
        self.ros_data_interface[v_id][data_name] = copy.deepcopy(msg)

    def inference(self, req, vehicle_id):
        try:
            while True:
                if self.env.world.action_step_flag != self.infer_step_flag:
                    break
                time.sleep(0.02)
            action = self.env.world.vehicle_list[0].state.action
            self.infer_step_flag = self.env.world.action_step_flag

        except rospy.ServiceException as exc:
            print("Failed to get obs vector, " + str(exc))
            action = None
        return inferResponse(action)

    def core_function(self):
        self.start_time = rospy.get_time()
        rate = rospy.Rate(self.core_fps)
        while self.env.world.real_world_reset == False and self.host_close == False:  # 可中断
            old_movable_list = copy.deepcopy(
                [v.state.movable for v in self.env.world.vehicle_list])
            total_time = rospy.get_time() - self.start_time
            self._update_data_interface()
            self.env.ros_step(total_time)
            for v_idx in range(self.agent_num):
                v = self.env.world.vehicle_list[v_idx]
                if v.vehicle_id != self.car_id:
                    continue
                if not(v.state.movable == old_movable_list[v_idx]):
                    if v.state.crashed:
                        rospyinfo('%s crashed' % v.vehicle_id)
                # send information ,also as  the connection-check
                sup_arg = supRequest()
                sup_arg.start = True
                sup_arg.movable = v.state.movable
                sup_arg.collision = v.state.crashed
                try:
                    self.client_ctrl_srv(sup_arg)
                except rospy.ServiceException as exc:
                    rospy.loginfo("Robot client %s Service Close: %s" %
                                  (v.vehicle_id, str(exc)))

            self.env.world.real_world_connection = True  # 在此表示初始化成功
            rate.sleep()
        # 退出并重启
        self.state_flag = 'reset'

    def _update_data_interface(self):
        for vehicle_idx in range(self.agent_num):
            vehicle_id = self.vehicle_id_list[vehicle_idx]
            data_interface = self.env.world.data_interface[vehicle_id]
            ros_data_interface = self.ros_data_interface[vehicle_id]
            if vehicle_id == self.car_id:
                data_interface['x'] = ros_data_interface['pose'].twist.linear.x
                data_interface['y'] = ros_data_interface['pose'].twist.linear.y
                data_interface['theta'] = ros_data_interface['pose'].twist.angular.z
                # N_laser = len(ros_data_interface['scan'].ranges)
                if self.env.world.lidar_on == True:
                    rmax = self.env.world.vehicle_list[vehicle_idx].lidar.range_max

                    for idx, data in enumerate(ros_data_interface['scan'].ranges):
                        rmax = self.env.world.vehicle_list[vehicle_idx].lidar.range_max
                        data = float(data)
                        if data > rmax:
                            data_interface['lidar'][(idx+360) % 720] = rmax
                        else:
                            data_interface['lidar'][(idx+360) % 720] = data
                if self.env.world.camera_on == True:
                    try:
                        data_interface['camera'] = self.bridge_.imgmsg_to_cv2(
                            ros_data_interface['camera/image_raw'], 'bgr8')
                    except CvBridgeError as err:
                        rospy.logerr(err)
            else:
                pass
                # No longer get other car's pose
                # pose = ros_data_interface['share_info'].data.coordinate[:3]
                # data_interface['x'] = pose[0]
                # data_interface['y'] = pose[1]
                # data_interface['theta'] = pose[2]
    def store_share_data(self,msg,args):
        v_id = args[0]
        data_name = args[1]
        self.share_info_dict[v_id][data_name] = copy.deepcopy(msg)

    def Dynamic_Detection(self):
        while True:
            node_name_list: list[str] = rosnode.get_node_names()
            robot_id_list = []
            for node_name in node_name_list:
                if node_name.endswith('robot_host') and (node_name.split('/')[-2])!=self.car_id:
                    # assume all robot_host note named as '/XXXX/vehicle_id/robot_host'
                    robot_id_list.append(node_name.split('/')[-2])
            # new id
            for robot_id in robot_id_list:
                with self.share_info_dict_lock:
                    if robot_id not in self.share_info_dict.keys():
                        self.share_info_dict[robot_id]={
                            "share_info":None,
                            "sub":None
                        }
                        topic_name = '/'+robot_id+'/'+"share_info"
                        data_class = rostopic.get_topic_class(topic_name)[0]
                        sub = rospy.Subscriber(name=topic_name,
                                       data_class=data_class,
                                       callback=self.store_share_data,
                                       callback_args=(robot_id, "share_info"))
                        self.share_info_dict[robot_id]["sub"]=sub
            # old id
            with self.share_info_dict_lock:
                for robot_id in self.share_info_dict.keys():
                    if robot_id not in robot_id_list:
                        sub=self.share_info_dict[robot_id]["sub"]
                        sub.unregister()
                        self.share_info_dict.pop(robot_id)
                    
            time.sleep(0.1)


    def GetShareInfo(self,DATA=None):
        share = share_info()
        share.car_id = self.car_id
        share.step = self.step_num

        data = share_data()
        if DATA ==None:
            data.step = -1
            #data.DATA=to_bytes( pickle.dumps(coordinate)) 
        else:
            data.step = self.step_num
            #data.DATA=to_bytes(pickle.dumps(DATA))
            data.DATA=DATA
        data_interface = self.env.world.data_interface[self.car_id]
        coordinate = [data_interface['x'],
                        data_interface['y'], data_interface['theta']]
        data.coordinate=coordinate
        share.data = data
        return share

    def step(self, action):
        # TODO: should check the class of action_n first
        # action的convert转换，包括目标类型（方向+距离）和持续控制类型（速度+持续时间）
        #
        # goal类型：由于是local goal，不是全局坐标，为了减小累积误差，限定车身方向只能为8个之一
        if action.type == "goal" and self.env.world.vehicle_list[0].robot_type == 'Mecanum':
            target_action = np.array([0, 0, 0])
            target_pose = None

            vehicle_id = self.car_id
            data_interface = self.env.world.data_interface[vehicle_id]
            if action.direction >= 0:

                theta_ctrl = action.direction*45
                theta = int(data_interface['theta']/np.pi*180)
                target_theta = round(
                    np.mod(theta+theta_ctrl, 360)/45)*45  # 0~8 车朝向限定为八个方位
            else:
                target_theta = action.yaw
                theta = int(data_interface['theta']/np.pi*180)
                theta_ctrl = np.mod(target_theta-theta, 360)  # 不限制在八个方位

            if action.distance >= 0:
                # 坐标系转换:local->world
                rel_target_theta = theta_ctrl/180*np.pi
                rel_target_x = action.distance*np.cos(rel_target_theta)
                rel_target_y = action.distance*np.sin(rel_target_theta)

                theta = theta/180*np.pi
                target_x, target_y = RotateAxis(
                    rel_target_x, rel_target_y, -theta)
                target_x += data_interface['x']
                target_y += data_interface['y']
            else:
                target_x, target_y = action.position
            target_pose = [target_theta, target_x, target_y]

            # 简单执行旋转再向前
            while True:
                flag = 0
                for i in range(1):
                    vehicle_id = self.car_id
                    data_interface = self.env.world.data_interface[vehicle_id]
                    target_theta, target_x, target_y = target_pose
                    # delta_theta  先掉头(可去掉)
                    theta = int(data_interface['theta']/np.pi*180)
                    delta_theta = target_theta-np.mod(theta, 360)
                    if abs(delta_theta) > 180:
                        delta_theta = delta_theta+360 if delta_theta <= -180 else delta_theta-360
                    if abs(delta_theta) > 3:
                        delta_theta = clip(delta_theta, 90, -90, 8)
                        target_action = np.array([
                            0, 0, delta_theta/180*np.pi*1.5])
                        # print(f'[{vehicle_id}]  delta_theta: {delta_theta}')
                        continue

                    # delta_distance  目标点的坐标系转换：world->local
                    related_x = target_x-data_interface['x']
                    related_y = target_y-data_interface['y']
                    related_theta = data_interface['theta']
                    delta_rel_x, delta_rel_y = RotateAxis(
                        related_x, related_y, related_theta)
                    if abs(delta_rel_x) <= 0.025 and abs(delta_rel_y) <= 0.025:
                        flag += 1
                        target_action = np.array([0, 0, 0])
                        continue
                    delta_rel_x = clip(delta_rel_x)
                    delta_rel_y = clip(delta_rel_y)
                    target_action = np.array([
                        delta_rel_x*1.6, delta_rel_y*1.6, 0])
                    # print(f'[{vehicle_id}]   delta_rel_x: {delta_rel_x} , delta_rel_y: {delta_rel_y}')

                raw_obs_n, _, _, _ = self.env.step([target_action])

                if flag == 1:
                    if self.env.world.vehicle_list[0].robot_type == 'Mecanum':
                        for i in range(self.agent_num):
                            if self.car_id == self.env.world.vehicle_list[i].vehicle_id:
                                obs = MecObs(
                                    x=raw_obs_n[i]['x'], y=raw_obs_n[i]['y'], yaw=raw_obs_n[i]['theta'])
                    break
                time.sleep(0.05)

            # 阻塞，保证同步
            self.step_num += 1
            while True:
                sync_flag = True
                self.share_pub_.publish(self.GetShareInfo())
                with self.share_info_dict_lock:
                    for v_id, inter in self.share_info_dict.items():
                        step = inter['share_info'].step
                        if step < self.step_num:
                            # print(f"{v_id} 's step_num too small ",step,self.step_num)
                            sync_flag = False
                            break
                if sync_flag:
                    break
                time.sleep(0.05)

        share_data_list = [obs]       
        for v_id, inter in self.ros_data_interface.items():
            if v_id == self.car_id:
                continue  
            #coordinate = pickle.loads(  from_bytes( inter['share_info'].data.DATA ) ) 
            #coordinate=json.loads(inter['share_info'].data.DATA)
            coordinate=inter['share_info'].data.coordinate
            share_data_list.append(MecObs(
                float(coordinate[0]), float(coordinate[1]), float(coordinate[2])))
        return share_data_list

        #return obs, None, None, None

    def reset(self):
        raw_obs_n = self.env.reset()  # TODO: put these codes in xxx_scenarios.py
        if self.env.world.vehicle_list[0].robot_type == 'Mecanum':
            for i in range(self.agent_num):
                if self.car_id == self.env.world.vehicle_list[i].vehicle_id:
                     obs=MecObs(x=raw_obs_n[i]['x'], y=raw_obs_n[i]['y'], yaw=raw_obs_n[i]['theta'])
        # 阻塞，保证同步
        self.step_num += 1
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
            time.sleep(0.05)

        share_data_list = [obs]       
        for v_id, inter in self.ros_data_interface.items():
            if v_id == self.car_id:
                continue
            #coordinate = pickle.loads( from_bytes(inter['share_info'].data.DATA) )
            coordinate=inter['share_info'].data.coordinate
            share_data_list.append(MecObs(
                float(coordinate[0]), float(coordinate[1]), float(coordinate[2])))
        return share_data_list

    def share(self, data):
        # TODO:
        # convert this more flexible and codes should in xxx_scenario.py
        share_ = self.GetShareInfo(DATA=data)
        share_data_list = []
        while True:
            sync_flag = True
            self.share_pub_.publish(share_)
            for v_id, inter in self.ros_data_interface.items():
                if v_id == self.car_id:
                    continue
                data = inter['share_info'].data
                if data.step < self.step_num:
                    sync_flag = False
                    break
            if sync_flag:
                break
            time.sleep(0.05)

        for v_id, inter in self.ros_data_interface.items():
            if v_id == self.car_id:
                continue
            #DATA =  pickle.loads(from_bytes( inter['share_info'].data.DATA) )
            DATA=inter['share_info'].data.DATA
            share_data_list.append(DATA)
        return share_data_list

    def close(self):
        self.host_close = True
        return self.env.close()


def add_config(parser):
    # not use now  please add in (for example   "gridworld/base/config.py")

    # 注意防止与上层的命名冲突
    # for deployment
    # parser.add_argument('--num_agents', type=int,
    #                    default=1, help="number of players")

    parser.add_argument('--real_world', default=True, action='store_true')
    parser.add_argument('--deploy_scenario_name', type=str,
                        default='deploy_scenario')
    # 根据小车类型选择 Ackerman  Mecanum
    parser.add_argument('--robot_type', type=str, default='Mecanum')
    parser.add_argument('--lidar_on', default=False, action='store_true')
    parser.add_argument('--camera_on', default=False, action='store_true')
    parser.add_argument('--collision_check', default=False,
                        action='store_true')  # 检测碰撞
    parser.add_argument('--init_pose_check', default=False,
                        action='store_true')  # 指定agent的初始位置
    return parser
