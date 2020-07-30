"""
Optimal_Bidirectional_RRT 2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils, queue


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class OBiRrt:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.x_init = Node(x_start)
        self.x_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.Ta = [self.x_init]
        self.Tb = [self.x_goal]
        self.c_best = np.inf
        self.sigma_best = {}
        self.path = []
        self.visited = []

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()
        # self.fig, self.ax = plt.subplots()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for k in range(self.iter_max):
            x_rand = self.generate_random_node(self.x_goal, self.goal_sample_rate)
            x_nearest = self.nearest_neighbor(self.Ta, x_rand)
            x_new = self.steer(x_nearest, x_rand)
            X_near_ind = self.nearest_neighbor(self.Ta, x_new)
            L_near = []

            for x_near_ind in X_near_ind:
                x_near = self.Ta[x_near_ind]
                sigma_near = self.steer(x_near, x_new)
                c_near = self.cost(x_near) + self.cost(sigma_near)
                L_near.append((c_near, x_near, sigma_near))

            L_near.sort()

    def cost_to_go(self, x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    def generate_random_node(self, goal_point, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return goal_point

    @staticmethod
    def nearest_neighbor(V, node):
        return V[int(np.argmin([math.hypot(nd.x - node.x, nd.y - node.y) for nd in V]))]

    def steer(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def find_near_neighbor(self, V, node):
        n = len(V) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in V]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node, V[ind])]

        return dist_table_index

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
