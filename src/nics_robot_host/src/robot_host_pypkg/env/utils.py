#!/usr/bin/python3
# -*- coding:utf-8 -*-

import math
import numpy as np
import sys
import json

def RPY2Quar(theta):

    cy = math.cos(theta * 0.5)
    sy = math.sin(theta * 0.5)
    cp = math.cos(0)
    sp = math.sin(0)
    cr = math.cos(0)
    sr = math.sin(0)

    ow = cy * cp * cr + sy * sp * sr
    ox = cy * cp * sr - sy * sp * cr
    oy = sy * cp * sr + cy * sp * cr
    oz = sy * cp * cr - cy * sp * sr
    return ow, ox, oy, oz


def clip(x, high=0.14, low=-0.14, bound=0.04,k=1.6,speed=None):
    y = max(low, min(1.5*x, high))
    if abs(y) > bound:
        pass
    elif y > 0:
        y = bound
    else:
        y = -bound
    y*=k
    if speed==None:
        return y
    else:
        return y+(speed-y)*(-0.25)  # pid


def GetVelocityCoefficient(car_id):
    # for each car, same velocity control can bring different results
    # we may need give it a specific coefficient
    Cases={
        # 1.0,1.0,1.0   straight:turn l/r :  1:1.6+
        # 0.7,0.7,1.4  for two type action cost: 1:1
        # 0.6 0.6 1.5   straight:turn l/r :  1.6:1
        "MKN_1":[0.6,0.6,1.5],     
        "MKN_2":[0.6,0.6,1.5],
        "MKN_3":[1.0,1.0,0.9],
        "MKN_4":[1.3,1.3,0.18], #TODO
        "MKN_5":[1.3,1.3,0.18],
    } 
    if car_id not in Cases.keys():
         raise Exception('[Error]: car_id not in known id_lists, and can\'t get its speed coefficient! ')
    return np.array(Cases[car_id])

def RotateAxis(raw_x, raw_y, theta):
    '''
    rotate anticlockwise : theta(rad)
    '''
    target_x = raw_x * np.cos(theta)+raw_y*np.sin(theta)
    target_y = -raw_x * np.sin(theta)+raw_y*np.cos(theta)
    return target_x, target_y

class Utils():
    def RPY2Quar(self,theta):

        cy = math.cos(theta * 0.5)
        sy = math.sin(theta * 0.5)
        cp = math.cos(0)
        sp = math.sin(0)
        cr = math.cos(0)
        sr = math.sin(0)

        ow = cy * cp * cr + sy * sp * sr
        ox = cy * cp * sr - sy * sp * cr
        oy = sy * cp * sr + cy * sp * cr
        oz = sy * cp * cr - cy * sp * sr
        return ow, ox, oy, oz


    def clip(self,x, high=0.14, low=-0.14, bound=0.04,k=1.6,speed=None):
        y = max(low, min(1.5*x, high))
        if abs(y) > bound:
            pass
        elif y > 0:
            y = bound
        else:
            y = -bound
        y*=k
        if speed==None:
            return y
        else:
            return y+(speed-y)*(-0.25)  # pid


    def GetVelocityCoefficient(self,car_id):
        # for each car, same velocity control can bring different results
        # we may need give it a specific coefficient
        Cases={
            # 1.0,1.0,1.0   straight:turn l/r :  1:1.6+
            # 0.7,0.7,1.4  for two type action cost: 1:1
            # 0.6 0.6 1.5   straight:turn l/r :  1.6:1
            "MKN_1":[0.6,0.6,1.5],     
            "MKN_2":[0.6,0.6,1.5],
            "MKN_3":[1.0,1.0,0.9],
            "MKN_4":[1.3,1.3,0.18], #TODO
            "MKN_5":[1.3,1.3,0.18],
        } 
        if car_id not in Cases.keys():
            raise Exception('[Error]: car_id not in known id_lists, and can\'t get its speed coefficient! ')
        return np.array(Cases[car_id])

    def RotateAxis(self,raw_x, raw_y, theta):
        '''
        rotate anticlockwise : theta(rad)
        '''
        target_x = raw_x * np.cos(theta)+raw_y*np.sin(theta)
        target_y = -raw_x * np.sin(theta)+raw_y*np.cos(theta)
        return target_x, target_y    


