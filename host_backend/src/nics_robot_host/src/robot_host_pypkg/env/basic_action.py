#!/usr/bin/env python
# -*- coding:utf-8 -*-


from typing import Tuple, List


class DiscreteAction:
    __slots__ = [
        'type',
        'direction',
        'yaw',
        'distance',
        'position',
    ]

    def __init__(self, type:str='discrete', direction:int=0, yaw:int=0, distance:float=0.0, position:Tuple[float, float]=[.0,.0]):
        self.type= type
        # 0~35, related direction offset (unit: 10 degrees)
        self.direction = direction
        # 0~360, if use absolute yaw, set direction < 0 (unit: 1 degree)
        self.yaw= yaw
        # 0.0~4.0, related distance offset
        self.distance = distance
        # If use absolute position ,set distance < 0
        self.position = position


class ContinuousAction:
    __slots__ = [
        'type',
        'forward',
        'left',
        'anticlockwise'
    ]

    def __init__(self, type:str='speed',forward: float = 1.0, left: float = 1.0, anticlockwise: float = 1.0):
        self.type=type
        # -np.inf ~ +np.inf , relative coefficient for  forward speed
        self.forward = forward
        # -np.inf ~ +np.inf , relative coefficient for left speed
        self.left = left
        # -np.inf ~ +np.inf , relative coefficient for anticlockwise speed
        self.anticlockwise = anticlockwise
