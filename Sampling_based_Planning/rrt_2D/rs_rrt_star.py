"""
RS_RRT_STAR_SMART 2D
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils
import CurvesGenerator.reeds_shepp as rs


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

    def planning(self):

        for k in range(self.iter_max):
            node_rand = self.Sample()
            node_nearest = self.Nearest(self.V, node_rand)
            node_new = self.Steer(node_nearest, node_rand)

    def Steer(self, node_start, node_end):
        sx, sy, syaw = node_start.x, node_start.y, node_start.yaw
        gx, gy, gyaw = node_end.x, node_end.y, node_end.yaw
        maxc = self.curv

        path = rs.calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=0.2)

        if not path:
            return None

        node_new = Node(path.x[-1], path.y[-1], path.yaw[-1])
        node_new.path_x = path.x
        node_new.path_y = path.y
        node_new.path_yaw = path.yaw
        node_new.cost = path.L
        node_new.parent = node_start

        return node_new

    def Sample(self):
        delta = self.utils.delta

        rnd = Node(random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                   random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
                   random.uniform(-math.pi, math.pi))

        return rnd

    @staticmethod
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]

