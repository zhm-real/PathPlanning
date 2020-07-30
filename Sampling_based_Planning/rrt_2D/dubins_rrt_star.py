"""
DUBINS_RRT_STAR 2D
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils
import CurvesGenerator.dubins_path as dubins


class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.paty_yaw = []
        self.parent = None
        self.cost = 0.0


class RrtStarSmart:
    def __init__(self, sx, sy, syaw, gx, gy, gyaw, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.s_start = Node(sx, sy, syaw)
        self.s_goal = Node(gx, gy, gyaw)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.curv = 1.0

        self.env = env.Env()
        self.utils = utils.Utils()

        self.fig, self.ax = plt.subplots()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.V = [self.s_start]
        self.path = None
