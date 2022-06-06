#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import argparse
import gym
import numpy as np
import time
import cv2

from robot_host_pypkg.robot_localhost import RobotHost, add_config
from robot_host_pypkg.env.basic_action import ContinuousAction,DiscreteAction
from robot_host_pypkg.env.basic_observation import ObservationWithRGBD



class SimpleAgent(object):
    # simple case
    def __init__(self):
        pass

    def choose_action(self, obs:ObservationWithRGBD ):
        ###################################### Test for image #############################################
        # obs.depth  shape: (480,640)    range:0.5m ~8m ,单位毫米 

        image=obs.img.astype(np.uint8)
        cv2.imwrite('./test_img.jpg',image)

        depth=np.array(obs.depth,dtype=np.float32)
        # norm_image = cv2.normalize(depth, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        im_color=cv2.applyColorMap(cv2.convertScaleAbs(depth,alpha=0.1),cv2.COLORMAP_JET)
        cv2.imwrite('./test_depth.jpg',im_color)

        #######################################Test for action ###########################################
        str_list = str(input("direction,distance:")).split(" ")
        try:
            action = DiscreteAction('discrete', float(
                str_list[0]), 0, float(
                str_list[1]), (0, 0))
        except:
            print("INPUT ERROR")
            action = DiscreteAction('discrete', 0, 0, 0, (0, 0))
        
        '''
        str_list = str(input("ratio for speed [x y yaw]: ")).split(" ")
        try:
            action = ContinuousAction('speed', float(str_list[0]),
                                    float(str_list[1]), 
                                    float(str_list[2]))
        except:
            print("INPUT ERROR")
            action = ContinuousAction('speed', 0, 0, 0)
        '''
        return action


class SimpleEnv(gym.Env):
    # simple case
    def __init__(self, host):
        self.host = host

    def step(self, action):
        return self.host.step(action)

    def reset(self):
        return self.host.reset()

    def share(self, data):
        return self.host.share(data)


def make_render_env(all_args):
    env = SimpleEnv(RobotHost(all_args))
    return env


def parse_args(args, parser):
    parser = add_config(parser)  # more argument details see this
    parser.add_argument('--num_agents', type=int,
                        default=1, help="number of players")
    all_args = parser.parse_known_args(args)[0]
    return all_args


def main(args):
    parser = argparse.ArgumentParser()
    all_args = parse_args(args, parser)
    env = make_render_env(all_args)
    agent = SimpleAgent()
    try :
        for i_seed in range(2):
            obs = env.reset()  
            for step_i in range(10):
                action = agent.choose_action(obs)
                obs = env.step(action)
                print(f'STEP {step_i}  FINISHED')
                # time.sleep(2)
    finally:
        env.step(ContinuousAction('speed', 0, 0, 0))
        env.close()


if __name__ == "__main__":
    main(sys.argv[1:])
