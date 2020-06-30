# this is the three dimensional near-sighted 1 neighborhood LRTA* algo
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
from Search_3D import Astar3D
from Search_3D.utils3D import getAABB, getDist, getRay, StateSpace, Heuristic, getNearest, isCollide, hash3D, dehash, \
    cost, obstacleFree
from Search_3D.plot_util3D import visualization
import queue

class LRT_A_star2:
    def __init__(self, resolution=0.5, N=7):
        self.N = N
        self.Astar = Astar3D.Weighted_A_star(resolution=resolution)
        self.path = []

    def children(self, x):
        allchild = []
        resolution = self.Astar.env.resolution
        for direc in self.Astar.Alldirec:
            child = np.array(list(map(np.add, x, np.multiply(direc, resolution))))
            allchild.append(hash3D(child))
        return allchild

    def updateHeuristic(self):
        # Initialize at infinity
        for strxi in self.Astar.CLOSED:
            self.Astar.h[strxi] = np.inf
        # initialize difference
        Diff = True
        while Diff:  # repeat until converge
            hvals, lasthvals = [], []
            for strxi in self.Astar.CLOSED:
                xi = dehash(strxi)
                lasthvals.append(self.Astar.h[strxi])
                # update h values if they are smaller
                minfval = min([cost(xi, xj, settings=0) + self.Astar.h[hash3D(xj)] for xj in self.Astar.children(xi)])
                if self.Astar.h[strxi] >= minfval:
                    self.Astar.h[strxi] = minfval
                hvals.append(self.Astar.h[strxi])
            if lasthvals == hvals: Diff = False

    def move(self):
        strst = self.Astar.x0
        st = self.Astar.start
        ind = 0
        while strst in self.Astar.CLOSED:  # when minchild in CLOSED then continue, when minchild in OPEN, stop
            # strChildren = self.children(st)
            strChildren = [hash3D(i) for i in self.Astar.children(st)]
            minh, minchild = np.inf, None
            for child in strChildren:
                h = self.Astar.h[child]
                if h <= minh:
                    minh, minchild = h, dehash(child)
            self.path.append([st, minchild])
            strst, st = hash3D(minchild), minchild
            for (_, strp) in self.Astar.OPEN.enumerate():
                if strp == strst:
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
    T = LRT_A_star2(resolution=0.5, N=1500)
    T.run()
