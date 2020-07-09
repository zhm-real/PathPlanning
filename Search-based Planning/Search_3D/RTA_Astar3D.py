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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, hash3D, dehash, \
    cost, obstacleFree
from Search_3D.plot_util3D import visualization
import queue

class RTA_A_star:
    def __init__(self, resolution=0.5, N=7):
        self.N = N # node to expand 
        self.Astar = Astar3D.Weighted_A_star(resolution=resolution) # initialize A star
        self.path = [] # empty path
        self.strst = []
        self.localhvals = []

    def updateHeuristic(self):
        # Initialize hvalues at infinity
        self.localhvals = []
        nodeset, vals = [], []
        for (_,strxi) in self.Astar.OPEN.enumerate():
            nodeset.append(strxi)
            vals.append(self.Astar.Space[strxi] + self.Astar.h[strxi])
        strj, fj = nodeset[np.argmin(vals)], min(vals)
        self.strst = strj
        # single pass update of hvals
        for strxi in self.Astar.CLOSED:
            # xi = dehash(strxi)
            self.Astar.h[strxi] = fj - self.Astar.Space[strxi]
            self.localhvals.append(self.Astar.h[strxi])
        
    def move(self):
        strst, localhvals = self.strst, self.localhvals
        maxhval = max(localhvals)
        st = dehash(strst)
        sthval = self.Astar.h[strst]
        # find the lowest path up hill
        while sthval < maxhval:
            parentsvals , parents = [] , []
            # find the max child
            for xi in self.Astar.children(st):
                strxi = hash3D(xi)
                if strxi in self.Astar.CLOSED:
                    parents.append(xi)
                    parentsvals.append(self.Astar.h[strxi])
            lastst = st            
            st, strst = parents[np.argmax(parentsvals)], hash3D(st)
            self.path.append([st,lastst]) # add to path
            sthval = self.Astar.h[strst]
        self.Astar.reset(dehash(self.strst))

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
    T = RTA_A_star(resolution=0.5, N=100)
    T.run()