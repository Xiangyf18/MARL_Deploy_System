#!/usr/bin/python2
# -*- coding: utf-8 -*-
import rospy

import numpy as np
import time
import threading
from geometry_msgs.msg import Twist
from nics_robot_host.srv import *
from nics_robot_host.msg import *


class RobotClient(object):
    def __init__(self, id_str):
        self.car_id = id_str
        rospy.init_node('robot_client')

        # get ros_param '/car_id/inference_fps'
        if rospy.has_param('inference_fps'):
            self.inference_fps = rospy.get_param('inference_fps')
        else:
            self.inference_fps = 50.0

        # get ros_param '/car_id/inference_fps'
        if rospy.has_param('send_velocity_fps'):
            self.send_velocity_fps = rospy.get_param('send_velocity_fps')
        else:
            self.send_velocity_fps = 50.0

        self.inference_twist = Twist()

        self.movable = 0
        self.run_state = 0

        self.client_control_thread = threading.Thread(
            target=self.client_control_target)
        self.client_control_thread.setDaemon(True)
        self.client_control_thread.start()

        while self.run_state == 0:
            time.sleep(0.1)

        rospy.loginfo('robot receive start signal')

        self.send_velocity_thread = threading.Thread(
            target=self.send_velocity_target)
        self.send_velocity_thread.setDaemon(True)

        self.inference_thread = threading.Thread(target=self.inference_target)
        self.inference_thread.setDaemon(True)

        self.send_velocity_thread.start()
        self.inference_thread.start()

        # check whether the connection have broken  (service:client-control)
        self.last_connection_time = rospy.get_time()
        while not rospy.is_shutdown():
            time_now = rospy.get_time()
            if (time_now-self.last_connection_time) >= 1:
                self.run_state = 0
                self.movable = 0
            time.sleep(0.1)

    def inference_target(self):

        self.inference_service_name = '/' + self.car_id + '/inference'
        rate = rospy.Rate(self.inference_fps)

        rospy.wait_for_service(self.inference_service_name, timeout=0.1)
        self.inference_client = rospy.ServiceProxy(
            self.inference_service_name, infer)

        inference_state = 'wait for service'

        while True:
            if inference_state == 'wait for service':
                try:
                    rospy.wait_for_service(
                        self.inference_service_name, timeout=0.1)
                except:
                    pass
                self.inference_client = rospy.ServiceProxy(
                    self.inference_service_name, infer)

            if self.run_state:
                try:
                    action = self.inference_client()
                    self.action_to_velocity(action.act_vector)
                    rate.sleep()
                except rospy.ServiceException as exc:
                    print("Can not get action: " + str(exc))
            else:
                rospy.sleep(0.1)

    def action_to_velocity(self, action):
        inference_twist=Twist()
        if self.car_id.split('_')[0] == 'AKM':
            inference_twist.linear.x = action[0]
            inference_twist.linear.y = 0
            inference_twist.linear.z = 0
            inference_twist.angular.x = 0
            inference_twist.angular.y = 0
            inference_twist.angular.z = action[1]
        elif self.car_id.split('_')[0] == 'MKN':
            inference_twist.linear.x = action[0]
            inference_twist.linear.y = action[1]
            inference_twist.linear.z = 0
            inference_twist.angular.x = 0
            inference_twist.angular.y = 0
            inference_twist.angular.z = action[2]
        else:
            inference_twist.linear.x = 0
            inference_twist.linear.y = 0
            inference_twist.linear.z = 0
            inference_twist.angular.x = 0
            inference_twist.angular.y = 0
            inference_twist.angular.z = 0
        self.inference_twist=inference_twist

    def send_velocity_target(self):
        """下端所接串口发送频率为 self.send_velocity_fps Hz，此线程不要有太多计算"""
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(self.send_velocity_fps)
        while True:
            if self.movable:
                self.pub.publish(self.inference_twist)
            else:
                send_twist = Twist()
                self.pub.publish(send_twist)
            rate.sleep()

    def client_control_target(self):
        self.car_allready = rospy.Service(
            'client_control', sup, self.client_control)
        rospy.loginfo("client_serv setup")
        self.car_allready.spin()

    def client_control(self, req):
        self.last_connection_time = rospy.get_time()
        self.run_state = req.start
        rospy.loginfo("client_req")
        if req.collision:
            self.movable = 0
        else:
            self.movable = req.movable
        rep = supResponse()
        rep.result = True
        rep.twist = self.inference_twist
        return rep
