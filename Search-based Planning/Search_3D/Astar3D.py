# this is the three dimensional A* algo
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, \
    cost, children, StateSpace
from Search_3D.plot_util3D import visualization
import queue
import time

class Weighted_A_star(object):
    def __init__(self, resolution=0.5):
        self.Alldirec = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
                                  [-1, 0, 0], [0, -1, 0], [0, 0, -1], [-1, -1, 0], [-1, 0, -1], [0, -1, -1],
                                  [-1, -1, -1],
                                  [1, -1, 0], [-1, 1, 0], [1, 0, -1], [-1, 0, 1], [0, 1, -1], [0, -1, 1],
                                  [1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]])

        self.env = env(resolution=resolution)
        self.X = StateSpace(self.env)
        self.g = g_Space(self)  # key is the point, store g value
        self.start, self.goal = getNearest(self.g, self.env.start), getNearest(self.g, self.env.goal)
        # self.AABB = getAABB(self.env.blocks)
        self.g[getNearest(self.g, self.start)] = 0  # set g(x0) = 0

        self.h = Heuristic(self.g, self.goal)
        self.Parent = {}
        self.CLOSED = set()
        self.V = []
        self.done = False
        self.Path = []
        self.ind = 0
        self.x0, self.xt = self.start, self.goal
        self.OPEN = queue.QueuePrior()  # store [point,priority]
        self.OPEN.put(self.x0, self.g[self.x0] + self.h[self.x0])  # item, priority = g + h
        self.lastpoint = self.x0

    # def children(self, x):
    #     allchild = []
    #     for j in self.Alldirec:
    #         collide, child = isCollide(self, x, j)
    #         if not collide:
    #             allchild.append(child)
    #     return allchild

    def run(self, N=None):
        xt = self.xt
        xi = self.x0
        while xt not in self.CLOSED and self.OPEN:  # while xt not reached and open is not empty
            xi = self.OPEN.get()
            if xi not in self.CLOSED:
                self.V.append(np.array(xi))
            self.CLOSED.add(xi)  # add the point in CLOSED set
            # visualization(self)
            allchild = children(self,xi)
            for xj in allchild:
                if xj not in self.CLOSED:
                    gi, gj = self.g[xi], self.g[xj]
                    a = gi + cost(self, xi, xj)
                    if a < gj:
                        self.g[xj] = a
                        self.Parent[xj] = xi
                        if (a, xj) in self.OPEN.enumerate():
                            # update priority of xj
                            self.OPEN.put(xj, a + 1 * self.h[xj])
                        else:
                            # add xj in to OPEN set
                            self.OPEN.put(xj, a + 1 * self.h[xj])
            # For specified expanded nodes, used primarily in LRTA*
            if N:
                if len(self.CLOSED) % N == 0:
                    break
            if self.ind % 100 == 0: print('number node expanded = ' + str(len(self.V)))
            self.ind += 1

        self.lastpoint = xi
        # if the path finding is finished
        if xt in self.CLOSED:
            self.done = True
            self.Path = self.path()
            # if N is None:
            #     visualization(self)
            #     plt.show()
            return True

        return False

    def path(self):
        path = []
        x = self.lastpoint
        start = self.x0
        while x != start:
            path.append([x, self.Parent[x]])
            x = self.Parent[x]
        # path = np.flip(path, axis=0)
        return path

    # utility used in LRTA*
    def reset(self, xj):
        self.g = g_Space(self)  # key is the point, store g value
        self.start = xj
        self.g[getNearest(self.g, self.start)] = 0  # set g(x0) = 0
        self.x0 = xj
        self.OPEN.put(self.x0, self.g[self.x0] + self.h[self.x0])  # item, priority = g + h
        self.CLOSED = set()

        # self.h = h(self.Space, self.goal)


if __name__ == '__main__':
    sta = time.time()
    Astar = Weighted_A_star(0.5)
    Astar.run()
    print(time.time() - sta)