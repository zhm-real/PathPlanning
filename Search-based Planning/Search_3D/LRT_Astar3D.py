# this is the three dimensional LRTA* algo
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


class LRT_A_star(object):
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
        self.h = Heuristic(self.Space,self.goal) # initialize heuristic
        self.Child = {}
        self.CLOSED = set()
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

    def step(self, xi, strxi):
        childs = self.children(xi) # find all childs within one move
        fvals = [cost(xi,i) + self.h[hash3D(i)] for i in childs]# f = g + h 
        xj , fmin = childs[np.argmin(fvals)], min(fvals)
        strxj = hash3D(xj)
        # add the child of xi
        self.Child[strxi] = xj
        if fmin >= self.h[strxi]: 
            self.h[strxi] = fmin # update h(xt) to f(xj) if f is greater
            # TODO: action to move to xj
            self.OPEN.put(strxj, fmin+1*self.h[strxj]) 

    def run(self):
        x0 = hash3D(self.start)
        xt = hash3D(self.goal)
        self.OPEN.put(x0, self.Space[x0] + self.h[x0]) # item, priority = g + h
        self.ind = 0
        while xt not in self.CLOSED and self.OPEN: # while xt not reached and open is not empty
            strxi = self.OPEN.get()           
            xi = dehash(strxi)
            self.CLOSED.add(strxi) # add the point in CLOSED set
            self.V.append(xi)
            visualization(self)
            self.step(xi , strxi)
            if self.ind % 100 == 0: print('iteration number = '+ str(self.ind))
            self.ind += 1
        self.done = True
        self.Path = self.path()
        visualization(self)
        plt.show()

    def path(self):
        # this is a suboptimal path. 
        path = []
        strgoal = hash3D(self.goal)
        strx = hash3D(self.start)
        ind = 0
        while strx != strgoal:
            path.append([dehash(strx),self.Child[strx]])
            strx = hash3D(self.Child[strx])
            ind += 1
            if ind == 1000:
                return np.flip(path,axis=0)
        path = np.flip(path,axis=0)
        return path

if __name__ == '__main__':
    Astar = LRT_A_star(0.5)
    Astar.run()