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
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isCollide, hash3D, dehash, cost
from Search_3D.plot_util3D import visualization
import queue


class Weighted_A_star(object):
    def __init__(self,resolution=0.5):
        self.Alldirec = np.array([[1 ,0,0],[0,1 ,0],[0,0, 1],[1 ,1 ,0],[1 ,0,1 ],[0, 1, 1],[ 1, 1, 1],\
                      [-1,0,0],[0,-1,0],[0,0,-1],[-1,-1,0],[-1,0,-1],[0,-1,-1],[-1,-1,-1],\
                      [1,-1,0],[-1,1,0],[1,0,-1],[-1,0, 1],[0,1, -1],[0, -1,1],\
                      [1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1]])
        self.env = env(resolution = resolution)
        self.Space = g_Space(self) # key is the point, store g value
        self.start, self.goal = getNearest(self.Space,self.env.start), getNearest(self.Space,self.env.goal)
        self.Space[hash3D(self.start)] = 0 # set g(x0) = 0
        self.Space[hash3D(self.goal)] = 0 # set g(x0) = 0
        self.OPEN1 = queue.QueuePrior() # store [point,priority]
        self.OPEN2 = queue.QueuePrior()
        self.h1 = Heuristic(self.Space,self.goal) # tree NO.1
        self.h2 = Heuristic(self.Space,self.start) # tree NO.2
        self.Parent1, self.Parent2 = {}, {}
        self.CLOSED1, self.CLOSED2 = set(), set()
        self.V = []
        self.done = False
        self.Path = []

    def children(self,x):
        allchild = []
        for j in self.Alldirec:
            collide,child = isCollide(self,x,j)
            if not collide:
                allchild.append(child)
        return allchild

    def run(self):
        x0, xt = hash3D(self.start), hash3D(self.goal)
        self.OPEN1.put(x0, self.Space[x0] + self.h1[x0]) # item, priority = g + h
        self.OPEN2.put(xt, self.Space[xt] + self.h2[xt]) # item, priority = g + h
        self.ind = 0
        while not self.CLOSED1.intersection(self.CLOSED2): # while xt not reached and open is not empty
            strxi1, strxi2 = self.OPEN1.get(), self.OPEN2.get() 
            xi1, xi2 = dehash(strxi1), dehash(strxi2)
            self.CLOSED1.add(strxi1) # add the point in CLOSED set
            self.CLOSED2.add(strxi2)
            self.V.append(xi1)
            self.V.append(xi2)
            visualization(self)
            allchild1,  allchild2 = self.children(xi1), self.children(xi2)
            self.evaluation(allchild1,strxi1,xi1,conf=1)
            self.evaluation(allchild2,strxi2,xi2,conf=2)
            if self.ind % 100 == 0: print('iteration number = '+ str(self.ind))
            self.ind += 1
        self.common = self.CLOSED1.intersection(self.CLOSED2)
        self.done = True
        self.Path = self.path()
        visualization(self)
        plt.show()

    def evaluation(self, allchild, strxi, xi, conf):
        for xj in allchild:
            strxj = hash3D(xj)
            if conf == 1:
                if strxj not in self.CLOSED1:
                    gi, gj = self.Space[strxi], self.Space[strxj]
                    a = gi + cost(xi,xj)
                    if a < gj:
                        self.Space[strxj] = a
                        self.Parent1[strxj] = xi
                        if (a, strxj) in self.OPEN1.enumerate():
                            self.OPEN1.put(strxj, a+1*self.h1[strxj])
                        else:
                            self.OPEN1.put(strxj, a+1*self.h1[strxj])
            if conf == 2:
                if strxj not in self.CLOSED2:
                    gi, gj = self.Space[strxi], self.Space[strxj]
                    a = gi + cost(xi,xj)
                    if a < gj:
                        self.Space[strxj] = a
                        self.Parent2[strxj] = xi
                        if (a, strxj) in self.OPEN2.enumerate():
                            self.OPEN2.put(strxj, a+1*self.h2[strxj])
                        else:
                            self.OPEN2.put(strxj, a+1*self.h2[strxj])
            
    def path(self):
        # TODO: fix path
        path = []
        strgoal = hash3D(self.goal)
        strstart = hash3D(self.start)
        strx = list(self.common)[0]
        while strx != strstart:
            path.append([dehash(strx),self.Parent1[strx]])
            strx = hash3D(self.Parent1[strx])
        strx = list(self.common)[0]
        while strx != strgoal:
            path.append([dehash(strx),self.Parent2[strx]])
            strx = hash3D(self.Parent2[strx])
        path = np.flip(path,axis=0)
        return path

if __name__ == '__main__':
    Astar = Weighted_A_star(0.5)
    Astar.run()