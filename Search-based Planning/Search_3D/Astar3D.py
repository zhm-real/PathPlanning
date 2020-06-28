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
from Search_3D.utils3D import getAABB, getDist, getRay, StateSpace, Heuristic, getNearest, isCollide, hash3D, dehash, cost
from Search_3D.plot_util3D import visualization
import queue


class Weighted_A_star(object):
    def __init__(self,resolution=0.5):
        self.Alldirec = np.array([[1 ,0,0],[0,1 ,0],[0,0, 1],[1 ,1 ,0],[1 ,0,1 ],[0, 1, 1],[ 1, 1, 1],\
                      [-1,0,0],[0,-1,0],[0,0,-1],[-1,-1,0],[-1,0,-1],[0,-1,-1],[-1,-1,-1],\
                      [1,-1,0],[-1,1,0],[1,0,-1],[-1,0, 1],[0,1, -1],[0, -1,1],\
                      [1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1]])
        self.env = env(resolution = resolution)
        self.Space = StateSpace(self) # key is the point, store g value
        self.start, self.goal = getNearest(self.Space,self.env.start), getNearest(self.Space,self.env.goal)
        self.AABB = getAABB(self.env.blocks)
        self.Space[hash3D(getNearest(self.Space,self.start))] = 0 # set g(x0) = 0
        self.OPEN = queue.QueuePrior() # store [point,priority]
        self.h = Heuristic(self.Space,self.goal)
        self.Parent = {}
        self.CLOSED = {}
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
        self.OPEN.put(x0, self.Space[x0] + self.h[x0]) # item, priority = g + h
        self.ind = 0
        while xt not in self.CLOSED and self.OPEN: # while xt not reached and open is not empty
            strxi = self.OPEN.get()           
            xi = dehash(strxi)
            self.CLOSED[strxi] = [] # add the point in CLOSED set
            self.V.append(xi)
            visualization(self)
            allchild = self.children(xi)
            for xj in allchild:
                strxj = hash3D(xj)
                if strxj not in self.CLOSED:
                    gi, gj = self.Space[strxi], self.Space[strxj]
                    a = gi + cost(xi,xj)
                    if a < gj:
                        self.Space[strxj] = a
                        self.Parent[strxj] = xi
                        if (a, strxj) in self.OPEN.enumerate():
                            # update priority of xj
                            self.OPEN.put(strxj, a+1*self.h[strxj])
                            pass
                        else:
                            # add xj in to OPEN set
                            self.OPEN.put(strxj, a+1*self.h[strxj])
            if self.ind % 100 == 0: print('iteration number = '+ str(self.ind))
            self.ind += 1
        self.done = True
        self.Path = self.path()
        visualization(self)
        plt.show()

    def path(self):
        path = []
        strx = hash3D(self.goal)
        strstart = hash3D(self.start)
        while strx != strstart:
            path.append([dehash(strx),self.Parent[strx]])
            strx = hash3D(self.Parent[strx])
        path = np.flip(path,axis=0)
        return path

if __name__ == '__main__':
    Astar = Weighted_A_star(0.5)
    Astar.run()