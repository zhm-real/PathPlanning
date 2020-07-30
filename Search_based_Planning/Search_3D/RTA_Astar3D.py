# this is the three dimensional Real-time Adaptive LRTA* algo
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

class RTA_A_star:
    def __init__(self, resolution=0.5, N=7):
        self.N = N # node to expand 
        self.Astar = Astar3D.Weighted_A_star(resolution=resolution) # initialize A star
        self.path = [] # empty path
        self.st = []
        self.localhvals = []

    def updateHeuristic(self):
        # Initialize hvalues at infinity
        self.localhvals = []
        nodeset, vals = [], []
        for (_,_,xi) in self.Astar.OPEN.enumerate():
            nodeset.append(xi)
            vals.append(self.Astar.g[xi] + self.Astar.h[xi])
        j, fj = nodeset[np.argmin(vals)], min(vals)
        self.st = j
        # single pass update of hvals
        for xi in self.Astar.CLOSED:
            self.Astar.h[xi] = fj - self.Astar.g[xi]
            self.localhvals.append(self.Astar.h[xi])
        
    def move(self):
        st, localhvals = self.st, self.localhvals
        maxhval = max(localhvals)
        sthval = self.Astar.h[st]
        # find the lowest path up hill
        while sthval < maxhval:
            parentsvals , parents = [] , []
            # find the max child
            for xi in children(self.Astar,st):
                if xi in self.Astar.CLOSED:
                    parents.append(xi)
                    parentsvals.append(self.Astar.h[xi])
            lastst = st            
            st = parents[np.argmax(parentsvals)]
            self.path.append([st,lastst]) # add to path
            sthval = self.Astar.h[st]
        self.Astar.reset(self.st)

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
    T = RTA_A_star(resolution=1, N=100)
    T.run()