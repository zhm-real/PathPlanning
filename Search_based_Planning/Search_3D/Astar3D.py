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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")
from Search_3D.env3D import env
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, \
    cost, children, StateSpace, heuristic_fun
from Search_3D.plot_util3D import visualization
import queue
import time

class Weighted_A_star(object):
    def __init__(self, resolution=0.5):
        self.Alldirec = {(1, 0, 0): 1, (0, 1, 0): 1, (0, 0, 1): 1, \
                        (-1, 0, 0): 1, (0, -1, 0): 1, (0, 0, -1): 1, \
                        (1, 1, 0): np.sqrt(2), (1, 0, 1): np.sqrt(2), (0, 1, 1): np.sqrt(2), \
                        (-1, -1, 0): np.sqrt(2), (-1, 0, -1): np.sqrt(2), (0, -1, -1): np.sqrt(2), \
                        (1, -1, 0): np.sqrt(2), (-1, 1, 0): np.sqrt(2), (1, 0, -1): np.sqrt(2), \
                        (-1, 0, 1): np.sqrt(2), (0, 1, -1): np.sqrt(2), (0, -1, 1): np.sqrt(2), \
                        (1, 1, 1): np.sqrt(3), (-1, -1, -1) : np.sqrt(3), \
                        (1, -1, -1): np.sqrt(3), (-1, 1, -1): np.sqrt(3), (-1, -1, 1): np.sqrt(3), \
                        (1, 1, -1): np.sqrt(3), (1, -1, 1): np.sqrt(3), (-1, 1, 1): np.sqrt(3)}
        self.settings = 'NonCollisionChecking' # 'NonCollisionChecking' or 'CollisionChecking'                
        self.env = env(resolution=resolution)
        self.start, self.goal = tuple(self.env.start), tuple(self.env.goal)
        self.g = {self.start:0,self.goal:np.inf}
        self.Parent = {}
        self.CLOSED = set()
        self.V = []
        self.done = False
        self.Path = []
        self.ind = 0
        self.x0, self.xt = self.start, self.goal
        self.OPEN = queue.MinheapPQ()  # store [point,priority]
        self.OPEN.put(self.x0, self.g[self.x0] + heuristic_fun(self,self.x0))  # item, priority = g + h
        self.lastpoint = self.x0

    def run(self, N=None):
        xt = self.xt
        xi = self.x0
        while self.OPEN:  # while xt not reached and open is not empty
            xi = self.OPEN.get()
            if xi not in self.CLOSED:
                self.V.append(np.array(xi))
            self.CLOSED.add(xi)  # add the point in CLOSED set
            if getDist(xi,xt) < self.env.resolution:
                break
            # visualization(self)
            for xj in children(self,xi):
                # if xj not in self.CLOSED:
                if xj not in self.g:
                    self.g[xj] = np.inf
                else:
                    pass
                a = self.g[xi] + cost(self, xi, xj)
                if a < self.g[xj]:
                    self.g[xj] = a
                    self.Parent[xj] = xi
                    # assign or update the priority in the open
                    self.OPEN.put(xj, a + 1 * heuristic_fun(self, xj))
            # For specified expanded nodes, used primarily in LRTA*
            if N:
                if len(self.CLOSED) % N == 0:
                    break
            if self.ind % 100 == 0: print('number node expanded = ' + str(len(self.V)))
            self.ind += 1

        self.lastpoint = xi
        # if the path finding is finished
        if self.lastpoint in self.CLOSED:
            self.done = True
            self.Path = self.path()
            if N is None:
                visualization(self)
                plt.show()
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
        self.OPEN.put(self.x0, self.g[self.x0] + heuristic_fun(self,self.x0))  # item, priority = g + h
        self.CLOSED = set()

        # self.h = h(self.Space, self.goal)


if __name__ == '__main__':
    
    Astar = Weighted_A_star(0.5)
    sta = time.time()
    Astar.run()
    print(time.time() - sta)