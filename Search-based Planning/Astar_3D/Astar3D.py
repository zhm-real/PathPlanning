# this is the three dimensional A* algo
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Astar_3D.env3D import env
from Astar_3D.utils3D import getAABB, getDist, getRay, StateSpace, Heuristic, getNearest
import queue


class Weighted_A_star(object):
    def __init__(self):
        self.Alldirec = np.array([[1 ,0,0],[0,1 ,0],[0,0, 1],[1 ,1 ,0],[1 ,0,1 ],[0, 1, 1],[ 1, 1, 1],\
                      [-1,0,0],[0,-1,0],[0,0,-1],[-1,-1,0],[-1,0,-1],[0,-1,-1],[-1,-1,-1],\
                      [1,-1,0],[-1,1,0],[1,0,-1],[-1,0, 1],[0,1, -1],[0, -1,1],\
                      [1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1]])
        self.env = env()
        self.Space = StateSpace(self.env.boundary) # key is the point, store g value
        self.OPEN = queue.QueuePrior() # store [point,priority]
        self.start = getNearest(self.Space,self.env.start)
        self.goal = getNearest(self.Space,self.env.goal)
        self.h = Heuristic(self.Space,self.goal)
        self.Parent = {}
        self.CLOSED = {}
        

    def run(self):
        pass
if __name__ == '__main__':
    Astar = Weighted_A_star()
    