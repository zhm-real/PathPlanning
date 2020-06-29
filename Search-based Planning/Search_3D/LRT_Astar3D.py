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
    cost
from Search_3D.plot_util3D import visualization
import queue


# class LRT_A_star1(object):
#     def __init__(self,resolution=0.5):
#         self.Alldirec = np.array([[1 ,0,0],[0,1 ,0],[0,0, 1],[1 ,1 ,0],[1 ,0,1 ],[0, 1, 1],[ 1, 1, 1],\
#                       [-1,0,0],[0,-1,0],[0,0,-1],[-1,-1,0],[-1,0,-1],[0,-1,-1],[-1,-1,-1],\
#                       [1,-1,0],[-1,1,0],[1,0,-1],[-1,0, 1],[0,1, -1],[0, -1,1],\
#                       [1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1]])
#         self.env = env(resolution = resolution)
#         self.Space = StateSpace(self)
#         self.start, self.goal = getNearest(self.Space,self.env.start), getNearest(self.Space,self.env.goal)
#         self.AABB = getAABB(self.env.blocks)
#         self.Space[hash3D(getNearest(self.Space,self.start))] = 0 # this is g
#         self.OPEN = queue.QueuePrior() 
#         self.h = Heuristic(self.Space,self.goal) # 1. initialize heuristic h = h0
#         self.Child = {}
#         self.CLOSED = set()
#         self.V = []
#         self.done = False
#         self.Path = []

#     def children(self,x):
#         allchild = []
#         for j in self.Alldirec:
#             collide,child = isCollide(self,x,j)
#             if not collide:
#                 allchild.append(child)
#         return allchild

#     def step(self, xi, strxi):
#         childs = self.children(xi) # 4. generate depth 1 neighborhood S(s,1) = {s' in S | norm(s,s') = 1}
#         fvals = [cost(xi,i) + self.h[hash3D(i)] for i in childs]
#         xj , fmin = childs[np.argmin(fvals)], min(fvals) # 5. compute h'(s) = min(dist(s,s') + h(s'))
#         strxj = hash3D(xj)
#         # add the child of xi
#         self.Child[strxi] = xj
#         if fmin >= self.h[strxi]: # 6. if h'(s) > h(s) then update  h(s) = h'(s)
#             self.h[strxi] = fmin 
#         # TODO: action to move to xj
#         self.OPEN.put(strxj, self.h[strxj]) # 7. update current state s = argmin (dist(s,s') + h(s'))

#     def run(self):
#         x0 = hash3D(self.start)
#         xt = hash3D(self.goal)
#         self.OPEN.put(x0, self.Space[x0] + self.h[x0]) # 2. reset the current state
#         self.ind = 0
#         while xt not in self.CLOSED and self.OPEN: # 3. while s not in Sg do
#             strxi = self.OPEN.get()           
#             xi = dehash(strxi) 
#             self.CLOSED.add(strxi) 
#             self.V.append(xi)
#             visualization(self)
#             if self.ind % 100 == 0: print('iteration number = '+ str(self.ind))
#             self.ind += 1
#         self.done = True
#         self.Path = self.path()
#         visualization(self)
#         plt.show()

#     def path(self):
#         # this is a suboptimal path. 
#         path = []
#         strgoal = hash3D(self.goal)
#         strx = hash3D(self.start)
#         ind = 0
#         while strx != strgoal:
#             path.append([dehash(strx),self.Child[strx]])
#             strx = hash3D(self.Child[strx])
#             ind += 1
#             if ind == 1000:
#                 return np.flip(path,axis=0)
#         path = np.flip(path,axis=0)
#         return path

class LRT_A_star2:
    def __init__(self, resolution=0.5, N=7):
        self.Astar = Astar3D.Weighted_A_star(resolution)

        while True:
            self.Astar.run(N)

    def updateHeuristic(self):
        for strxi in self.Astar.CLOSED:
            self.Astar.h[strxi] = np.inf
            xi = dehash(strxi)
            self.Astar.h[strxi] = min([cost(xi, xj) + self.Astar.h[hash3D(xj)] for xj in self.Astar.children(xi)])

    def move(self):
        print(np.argmin([j[0] for j in self.Astar.OPEN.enumerate()]))


if __name__ == '__main__':
    T = LRT_A_star2(resolution=1, N=50)
