"""
INFORMED_RRT_STAR 2D
@author: huiming zhou
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling-based Planning/")

from rrt_2D import env
from rrt_2D import plotting
from rrt_2D import utils
from rrt_2D import queue


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class IRrtStar:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        # self.fig, self.ax = plt.subplots()
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.V = [self.x_start]
        self.X_soln = set()
        self.path = None

    def planning(self):
        c_best = np.inf
        c_min = self.Line(self.x_start, self.x_goal)
        x_center = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                             [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])
        a1 = np.array([[(self.x_goal.x - self.x_start.x) / c_min],
                       [(self.x_goal.y - self.x_start.y) / c_min], [0.0]])
        e_theta = math.atan2(a1[1], a1[0])
        id1_t = np.array([[1.0, 0.0, 0.0]])
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, True, True)
        C = np.dot(np.dot(U, np.diag(
            [1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)

        for k in range(self.iter_max):
            if k % 500 == 0:
                print(k)

            if self.X_soln:
                c_best = min([self.Cost(x) for x in self.X_soln])

            x_rand = self.Sample(self.x_start, self.x_goal, c_best, x_center, C)
            x_nearest = self.Nearest(self.V, x_rand)
            x_new = self.Steer(x_nearest, x_rand)

            if x_new and not self.utils.is_collision(x_nearest, x_new):
                self.V.append(x_new)
                X_near = self.Near(self.V, x_new)
                x_min = x_nearest
                c_min = self.Cost(x_min) + self.Line(x_nearest, x_new)

                for x_near in X_near:
                    c_new = self.Cost(x_near) + self.Line(x_near, x_new)
                    if c_new < c_min:
                        x_new.parent = x_near
                        c_min = c_new

                for x_near in X_near:
                    c_near = self.Cost(x_near)
                    c_new = self.Cost(x_new) + self.Line(x_new, x_near)
                    if c_new < c_near:
                        x_near.parent = x_new

                if self.InGoalRegion(x_new):
                    self.X_soln.add(x_new)

        path = self.ExtractPath(self.V[-1])
        self.plotting.animation(self.V, path, "Informed rrt*")

    def ExtractPath(self, node):
        path = [[self.x_goal.x, self.x_goal.y]]

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.x_start.x, self.x_start.y])

        return path

    def InGoalRegion(self, node):
        if self.Line(node, self.x_goal) < self.step_len:
            return True

        return False

    def Steer(self, x_start, x_goal):
        dist, theta = self.get_distance_and_angle(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new

    def Near(self, nodelist, node):
        n = len(nodelist) + 1
        r = 2 * self.search_radius * math.sqrt((math.log(n) / n))

        dist_table = [math.hypot(nd.x - node.x, nd.y - node.y) for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r and
                  not self.utils.is_collision(node, nodelist[ind])]

        return X_near

    def Sample(self, x_start, x_goal, c_max, x_center, C):
        if c_max < np.inf:
            c_min = self.Line(x_start, x_goal)
            r = [c_max / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            L = np.diag(r)
            x_ball = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), x_ball) + x_center
        else:
            x_rand = self.SampleFreeSpace()

        return x_rand

    def SampleFreeSpace(self):
        delta = self.utils.delta

        if np.random.random() > self.goal_sample_rate:
            return [np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)]

        return [self.x_goal.x, self.x_goal.y]

    @staticmethod
    def SampleUnitNBall():
        x, y = random.random(), random.random()

        if y < x:
            x, y = y, x

        sample = np.array([[y * math.cos(2 * math.pi * x / y)],
                           [y * math.sin(2 * math.pi * x / y)], [0.0]])

        return sample

    @staticmethod
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                       for nd in nodelist]))]

    @staticmethod
    def Line(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    @staticmethod
    def Cost(node_p):
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


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt_star = IRrtStar(x_start, x_goal, 10, 0.10, 20, 4000)
    rrt_star.planning()


if __name__ == '__main__':
    main()
