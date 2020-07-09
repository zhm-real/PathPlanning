# this is the three dimensional bidirectional A* algo
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, cost, children
from Search_3D.plot_util3D import visualization
import queue


class Weighted_A_star(object):
    def __init__(self,resolution=0.5):
        self.Alldirec = np.array([[1 ,0,0],[0,1 ,0],[0,0, 1],[1 ,1 ,0],[1 ,0,1 ],[0, 1, 1],[ 1, 1, 1],\
                      [-1,0,0],[0,-1,0],[0,0,-1],[-1,-1,0],[-1,0,-1],[0,-1,-1],[-1,-1,-1],\
                      [1,-1,0],[-1,1,0],[1,0,-1],[-1,0, 1],[0,1, -1],[0, -1,1],\
                      [1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1]])
        self.env = env(resolution = resolution)
        self.g = g_Space(self) # key is the point, store g value
        self.start, self.goal = getNearest(self.g,self.env.start), getNearest(self.g,self.env.goal)
        self.g[self.start] = 0 # set g(x0) = 0
        self.g[self.goal] = 0 # set g(x0) = 0
        self.OPEN1 = queue.QueuePrior() # store [point,priority]
        self.OPEN2 = queue.QueuePrior()
        self.h1 = Heuristic(self.g,self.goal) # tree NO.1
        self.h2 = Heuristic(self.g,self.start) # tree NO.2
        self.Parent1, self.Parent2 = {}, {}
        self.CLOSED1, self.CLOSED2 = set(), set()
        self.V = []
        self.done = False
        self.Path = []

    def run(self):
        x0, xt = self.start, self.goal
        self.OPEN1.put(x0, self.g[x0] + self.h1[x0]) # item, priority = g + h
        self.OPEN2.put(xt, self.g[xt] + self.h2[xt]) # item, priority = g + h
        self.ind = 0
        while not self.CLOSED1.intersection(self.CLOSED2): # while xt not reached and open is not empty
            xi1, xi2 = self.OPEN1.get(), self.OPEN2.get() 
            self.CLOSED1.add(xi1) # add the point in CLOSED set
            self.CLOSED2.add(xi2)
            self.V.append(xi1)
            self.V.append(xi2)
            visualization(self)
            allchild1,  allchild2 = children(self,xi1), children(self,xi2)
            self.evaluation(allchild1,xi1,conf=1)
            self.evaluation(allchild2,xi2,conf=2)
            if self.ind % 100 == 0: print('iteration number = '+ str(self.ind))
            self.ind += 1
        self.common = self.CLOSED1.intersection(self.CLOSED2)
        self.done = True
        self.Path = self.path()
        visualization(self)
        plt.show()

    def evaluation(self, allchild, xi, conf):
        for xj in allchild:
            if conf == 1:
                if xj not in self.CLOSED1:
                    gi, gj = self.g[xi], self.g[xj]
                    a = gi + cost(self,xi,xj)
                    if a < gj:
                        self.g[xj] = a
                        self.Parent1[xj] = xi
                        if (a, xj) in self.OPEN1.enumerate():
                            self.OPEN1.put(xj, a+1*self.h1[xj])
                        else:
                            self.OPEN1.put(xj, a+1*self.h1[xj])
            if conf == 2:
                if xj not in self.CLOSED2:
                    gi, gj = self.g[xi], self.g[xj]
                    a = gi + cost(self,xi,xj)
                    if a < gj:
                        self.g[xj] = a
                        self.Parent2[xj] = xi
                        if (a, xj) in self.OPEN2.enumerate():
                            self.OPEN2.put(xj, a+1*self.h2[xj])
                        else:
                            self.OPEN2.put(xj, a+1*self.h2[xj])
            
    def path(self):
        # TODO: fix path
        path = []
        goal = self.goal
        start = self.start
        x = list(self.common)[0]
        while x != start:
            path.append([x,self.Parent1[x]])
            x = self.Parent1[x]
        x = list(self.common)[0]
        while x != goal:
            path.append([x,self.Parent2[x]])
            x = self.Parent2[x]
        path = np.flip(path,axis=0)
        return path

if __name__ == '__main__':
    Astar = Weighted_A_star(0.5)
    Astar.run()