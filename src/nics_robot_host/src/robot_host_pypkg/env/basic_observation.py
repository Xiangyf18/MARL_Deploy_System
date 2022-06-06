#!/usr/bin/env python
# -*- coding:utf-8 -*-


from numpy import ndarray


class Observation:
    __slots__ = ['x', 'y', 'yaw']

    def __init__(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw


class ObservationWithLidar:
    __slots__ = ['x', 'y', 'yaw','ranges']

    def __init__(self, x: float, y: float, yaw: float, ranges: ndarray):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.ranges = ranges

 

class ObservationWithRGBD:
    __slots__ = ['x', 'y', 'yaw', 'img', 'depth']

    def __init__(self, x: float, y: float, yaw: float, img: ndarray, depth: ndarray):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.img = img
        self.depth = depth

