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
from Search_3D.utils3D import getAABB, getDist, getRay, StateSpace, Heuristic, getNearest, isCollide, hash3D, dehash, \
    cost
from Search_3D.plot_util3D import visualization
import queue


class Weighted_A_star(object):
    def __init__(self, resolution=0.5):
        self.Alldirec = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
                                  [-1, 0, 0], [0, -1, 0], [0, 0, -1], [-1, -1, 0], [-1, 0, -1], [0, -1, -1],
                                  [-1, -1, -1],
                                  [1, -1, 0], [-1, 1, 0], [1, 0, -1], [-1, 0, 1], [0, 1, -1], [0, -1, 1],
                                  [1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]])

        self.env = env(resolution=resolution)
        self.Space = StateSpace(self)  # key is the point, store g value
        self.start, self.goal = getNearest(self.Space, self.env.start), getNearest(self.Space, self.env.goal)
        self.AABB = getAABB(self.env.blocks)
        self.Space[hash3D(getNearest(self.Space, self.start))] = 0  # set g(x0) = 0

        self.h = Heuristic(self.Space, self.goal)
        self.Parent = {}
        self.CLOSED = set()
        self.V = []
        self.done = False
        self.Path = []
        self.ind = 0
        self.x0, self.xt = hash3D(self.start), hash3D(self.goal)
        self.OPEN = queue.QueuePrior()  # store [point,priority]
        self.OPEN.put(self.x0, self.Space[self.x0] + self.h[self.x0])  # item, priority = g + h
        self.lastpoint = self.x0

    def children(self, x):
        allchild = []
        for j in self.Alldirec:
            collide, child = isCollide(self, x, j)
            if not collide:
                allchild.append(child)
        return allchild

    def run(self, N=None):
        xt = self.xt
        strxi = self.x0
        while xt not in self.CLOSED and self.OPEN:  # while xt not reached and open is not empty
            strxi = self.OPEN.get()
            xi = dehash(strxi)
            if strxi not in self.CLOSED:
                self.V.append(xi)
            self.CLOSED.add(strxi)  # add the point in CLOSED set
            visualization(self)
            allchild = self.children(xi)
            for xj in allchild:
                strxj = hash3D(xj)
                if strxj not in self.CLOSED:
                    gi, gj = self.Space[strxi], self.Space[strxj]
                    a = gi + cost(xi, xj)
                    if a < gj:
                        self.Space[strxj] = a
                        self.Parent[strxj] = xi
                        if (a, strxj) in self.OPEN.enumerate():
                            # update priority of xj
                            self.OPEN.put(strxj, a + 1 * self.h[strxj])
                        else:
                            # add xj in to U set
                            self.OPEN.put(strxj, a + 1 * self.h[strxj])
            # For specified expanded nodes, used primarily in LRTA*
            if N:
                if len(self.CLOSED) % N == 0:
                    break
            if self.ind % 100 == 0: print('number node expanded = ' + str(len(self.V)))
            self.ind += 1

        self.lastpoint = strxi
        # if the path finding is finished
        if xt in self.CLOSED:
            self.done = True
            self.Path = self.path()
            if N is None:
                visualization(self)
                plt.show()
            return True

        return False

    def path(self):
        path = []
        strx = self.lastpoint
        # strstart = hash3D(getNearest(self.Space, self.env.start))
        strstart = self.x0
        while strx != strstart:
            path.append([dehash(strx), self.Parent[strx]])
            strx = hash3D(self.Parent[strx])
        # path = np.flip(path, axis=0)
        return path

    # utility used in LRTA*
    def reset(self, xj):
        self.Space = StateSpace(self)  # key is the point, store g value
        self.start = xj
        self.Space[hash3D(getNearest(self.Space, self.start))] = 0  # set g(x0) = 0
        self.x0 = hash3D(xj)
        self.OPEN.put(self.x0, self.Space[self.x0] + self.h[self.x0])  # item, priority = g + h
        self.CLOSED = set()

        # self.h = h(self.Space, self.goal)


if __name__ == '__main__':
    Astar = Weighted_A_star(1)
    Astar.run()
