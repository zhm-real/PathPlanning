# this is the three dimensional N>1 LRTA* algo
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
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, \
    cost, obstacleFree, children
from Search_3D.plot_util3D import visualization
import queue

class LRT_A_star2:
    def __init__(self, resolution=0.5, N=7):
        self.N = N
        self.Astar = Astar3D.Weighted_A_star(resolution=resolution)
        self.path = []

    def updateHeuristic(self):
        # Initialize hvalues at infinity
        for xi in self.Astar.CLOSED:
            self.Astar.h[xi] = np.inf
        Diff = True
        while Diff:  # repeat DP until converge
            hvals, lasthvals = [], []
            for xi in self.Astar.CLOSED:
                lasthvals.append(self.Astar.h[xi])
                # update h values if they are smaller
                Children = children(self.Astar,xi)
                minfval = min([cost(self.Astar,xi, xj, settings=0) + self.Astar.h[xj] for xj in Children])
                # h(s) = h(s') if h(s) > cBest(s,s') + h(s') 
                if self.Astar.h[xi] >= minfval:
                    self.Astar.h[xi] = minfval
                hvals.append(self.Astar.h[xi])
            if lasthvals == hvals: Diff = False

    def move(self):
        st = self.Astar.x0
        ind = 0
        # find the lowest path down hill
        while st in self.Astar.CLOSED:  # when minchild in CLOSED then continue, when minchild in OPEN, stop
            Children = children(self.Astar,st)
            minh, minchild = np.inf, None
            for child in Children:
                # check collision here, not a supper efficient
                collide, _ = isCollide(self.Astar,st, child)
                if collide:
                    continue
                h = self.Astar.h[child]
                if h <= minh:
                    minh, minchild = h, child
            self.path.append([st, minchild])
            st = minchild
            for (_, p) in self.Astar.OPEN.enumerate():
                if p == st:
                    break
            ind += 1
            if ind > 1000:
                break
        self.Astar.reset(st)

    def run(self):
        while True:
            if self.Astar.run(N=self.N):
                self.Astar.Path = self.Astar.Path + self.path
                self.Astar.done = True
                visualization(self.Astar)
                plt.show()
                break
            self.updateHeuristic()
            self.move()


if __name__ == '__main__':
    T = LRT_A_star2(resolution=0.5, N=100)
    T.run()
