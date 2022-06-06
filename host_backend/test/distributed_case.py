#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import argparse
import gym
import numpy as np
import time

from robot_host_pypkg.robot_localhost import RobotHost, add_config
from sim2real.hosts.mecanum import Action, Observation, Observation

# general action interfacexxl


class SimpleAgent(object):
    # simple case
    def __init__(self):
        pass

    def getGoal(self, obs):
        return "I have two"

    def choose_action(self, obs: Observation, share_obs: Observation):
        # print(np.array(obs['lidar']).shape)
        # print(np.array(obs['camera']).shape)
        str_list = str(input("direction,distance:")).split(" ")
        try:
            action = Action('goal', float(
                str_list[0]), 0, float(
                str_list[1]), (0, 0))
        except:
            print("INPUT ERROR")
            action = Action('goal', 0, 0, 0, (0, 0))
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
    for i_seed in range(100):
        obs = env.reset()  # TODO：  a  list
        share_obs = env.share(agent.getGoal(obs))
        for step_i in range(10):  # 0.5s*30=15s
            action = agent.choose_action(obs, share_obs)
            obs = env.step(action)
            print(f'STEP {step_i}  FINISHED')
            share_obs = env.share(agent.getGoal(obs))
            # time.sleep(2)

    env.close()


if __name__ == "__main__":
    main(sys.argv[1:])
