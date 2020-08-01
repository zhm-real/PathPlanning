"""
Batch Informed Trees (BIT*)
@author: huiming zhou
"""

import os
import sys
import math
import random
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Tree:
    def __init__(self, x_start, x_goal):
        self.x_start = x_start
        self.goal = x_goal

        self.r = np.inf
        self.V = set()
        self.E = set()
        self.QE = set()
        self.QV = set()

        self.V_old = set()


class BITStar:
    def __init__(self, x_start, x_goal, eta, iter_max):
        self.x_start = Node(x_start[0], x_start[1])
        self.x_goal = Node(x_goal[0], x_goal[1])
        self.eta = eta
        self.iter_max = iter_max

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils()

        self.fig, self.ax = plt.subplots()

        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range

        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.Tree = Tree(self.x_start, self.x_goal)
        self.X_sample = set()
        self.g_T = dict()
        self.f_T = dict()

    def init(self):
        self.Tree.V.add(self.x_start)
        self.X_sample.add(self.x_goal)
        self.g_T[self.x_goal] = np.inf
        self.f_T[self.x_goal] = 0.0
        self.g_T[self.x_start] = 0.0
        self.f_T[self.x_start] = self.f_estimated(self.x_start)

        cMin, theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])

        return theta, cMin, xCenter, C

    def planning(self):
        eTheta, cMin, xCenter, C = self.init()

        for k in range(self.iter_max):
            if not self.Tree.QE and not self.Tree.QV:
                self.Prune(self.g_T[self.x_goal])
                m = 200
                self.X_sample.update(self.Sample(m, self.g_T[self.x_goal], cMin, xCenter, C))
                self.Tree.V_old = copy.deepcopy(self.Tree.V)
                self.Tree.QV = copy.deepcopy(self.Tree.V)
                self.Tree.r = self.radius(len(self.Tree.V) + len(self.X_sample))

            while self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                self.ExpandVertex(self.BestInVertexQueue())

            vm, xm = self.BestInEdgeQueue()
            self.Tree.QE.remove((vm, xm))

            if self.g_T[vm] + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.g_T[self.x_goal]:
                if self.g_estimated(vm) + self.cost(vm, xm) + self.h_estimated(xm) < self.g_T[self.x_goal]:
                    if self.g_T[vm] + self.cost(vm, xm) < self.g_T[xm]:
                        if xm in self.Tree.V:
                            # remove edges
                            for vl, vr in self.Tree.E:
                                if vl == xm or vr == xm:
                                    self.Tree.E.remove((vl, vr))
                        else:
                            self.X_sample.remove(xm)
                            self.Tree.V.add(xm)
                            self.Tree.QV.add(xm)

                        self.Tree.E.add((vm, xm))

    def ExpandVertex(self, v):
        self.Tree.QV.remove(v)
        X_near = {x for x in self.X_sample if self.calc_dist(x, v) <= self.Tree.r}

        for x in X_near:
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.g_T[self.x_goal]:
                self.Tree.QE.add((v, x))

        if v not in self.Tree.V_old:
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}

            for w in V_near:
                if (v, w) not in self.Tree.E and (w, v) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.g_T[self.x_goal] and \
                        self.g_T[v] + self.calc_dist(v, w) < self.g_T[w]:
                    self.Tree.QE.add((v, w))

    def BestVertexQueueValue(self):
        if not self.Tree.QV:
            return np.inf

        return min(self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV)

    def BestEdgeQueueValue(self):
        if not self.Tree.QE:
            return np.inf

        return min(self.g_T[el] + self.calc_dist(el, er) + self.h_estimated(er)
                   for el, er in self.Tree.QE)

    def BestInVertexQueue(self):
        v_value = {v: self.g_T[v] + self.h_estimated(v) for v in self.Tree.QV}

        return min(v_value, key=v_value.get)

    def BestInEdgeQueue(self):
        e_value = {(el, er): self.g_T[el] + self.calc_dist(el, er) + self.h_estimated(er)
                   for el, er in self.Tree.QE}

        return min(e_value, key=e_value.get)

    def radius(self, q):
        lambda_X = 0
        sigma = math.pi ** 2

        for x in self.Tree.V:
            if self.f_estimated(x) <= self.g_T[self.x_goal]:
                lambda_X += 1

        return 2 * self.eta * 1.5 ** 0.5 * (lambda_X / sigma * math.log(q) / q) ** 0.5

    def Sample(self, m, cMax, cMin, xCenter, C):
        if cMax < np.inf:
            Sample = self.SampleEllipsoid(m, cMax, cMin, xCenter, C)
        else:
            Sample = self.SampleFreeSpace(m)

        return Sample

    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0,
             math.sqrt(cMax ** 2 - cMin ** 2) / 2.0]
        L = np.diag(r)

        ind = 0
        delta = self.delta
        Sample = set()

        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = C @ L @ xBall + xCenter
            node = Node(x_rand[0], x_rand[1])
            not_in_obs = ~self.utils.is_inside_obs(node)
            in_x_range = self.x_range[0] + delta <= node.x <= self.x_range[1] - delta
            in_y_range = self.y_range[0] + delta <= node.y <= self.y_range[1] - delta

            if not_in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1

        return Sample

    def SampleFreeSpace(self, m):
        delta = self.utils.delta
        Sample = set()

        ind = 0
        while ind < m:
            node = Node((random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1

        return Sample

    @staticmethod
    def SampleUnitNBall():
        theta, r = random.uniform(0.0, 2 * math.pi), random.random()
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        return np.array([[x], [y], [0.0]])

    def Prune(self, c):
        for x in self.X_sample:
            if self.f_estimated(x) >= c:
                self.X_sample.remove(x)

        for v in self.Tree.V:
            if self.f_estimated(v) > c:
                self.Tree.V.remove(v)

        for v, w in self.Tree.E:
            if self.f_estimated(v) > c or self.f_estimated(w) > c:
                self.Tree.E.remove((v, w))

        for v in self.Tree.V:
            if v.g_T == np.inf:
                self.X_sample.add(v)

        for v in self.Tree.V:
            if v.g_T == np.inf:
                self.Tree.V.remove(v)

    def cost(self, start, end):
        if self.utils.is_collision(start, end):
            return np.inf

        return self.calc_dist(start, end)

    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)

    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node)

    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal)

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_start.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C

    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.x - end.x, start.y - end.y)

    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
