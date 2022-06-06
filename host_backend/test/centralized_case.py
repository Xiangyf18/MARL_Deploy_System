#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import argparse
import gym
import numpy as np
import time

from robot_host_pypkg.robot_host_centralized import RobotHost, add_config
from sim2real.hosts.mecanum import Action as MecAction
# general action interfacexxl


class SimpleAgent(object):
    # simple case
    def __init__(self):
        pass

    def choose_action(self, obs):
        # print(np.array(obs['lidar']).shape)
        # print(np.array(obs['camera']).shape)
        str_list = str(input("direction,distance:")).split(" ")
        try:
            action = MecAction('goal', float(
                str_list[0]), 0, float(
                str_list[1]), (0, 0))
        except:
            print("INPUT ERROR")
            action = MecAction('goal', 0, 0, 0, (0, 0))
        return action


class SimpleEnv(gym.Env):
    # simple case
    def __init__(self, host):
        self.host = host

    def step(self, action_n):
        return self.host.step(action_n)

    def reset(self):
        return self.host.reset()


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
    agents = [SimpleAgent() for i in range(all_args.num_agents)]
    for i_seed in range(2):
        obs_n = env.reset()
        for step_i in range(10):  # 0.5s*30=15s
            action_n = []
            for i in range(all_args.num_agents):
                action_n.append(agents[i].choose_action(obs_n[i]))
            obs_n, reward_n, done_n, info_n = env.step(action_n)
            print("STEP FINISHED")
            # time.sleep(2)

    env.close()


if __name__ == "__main__":
    main(sys.argv[1:])
