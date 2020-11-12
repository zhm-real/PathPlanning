# Real-Time Randomized Path Planning for Robot Navigation∗
"""
This is rrt extend code for 3D
@author: yue qi
"""
import numpy as np
from numpy.matlib import repmat
from collections import defaultdict
import time
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path

# here attempt to use a KD tree for the data structure implementation
import scipy.spatial.kdtree as KDtree


class extend_rrt(object):
    def __init__(self):
        self.env = env()
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.current = tuple(self.env.start)
        self.stepsize = 0.5
        self.maxiter = 10000
        self.GoalProb = 0.05 # probability biased to the goal
        self.WayPointProb = 0.05 # probability falls back on to the way points

        self.done = False
        self.V = [] # vertices
        self.Parent = {}
        self.Path = []
        self.ind = 0
        self.i = 0

    #--------- basic rrt algorithm----------
    def RRTplan(self, env, initial, goal):
        threshold = self.stepsize
        nearest = initial # state structure
        self.V.append(initial)
        rrt_tree = initial # TODO KDtree structure
        while self.ind <= self.maxiter:
            target = self.ChooseTarget(goal)
            nearest = self.Nearest(rrt_tree, target)
            extended, collide = self.Extend(env, nearest, target)
            if not collide:
                self.AddNode(rrt_tree, nearest, extended)
                if getDist(nearest, goal) <= threshold:
                    self.AddNode(rrt_tree, nearest, self.xt)
                    break
                self.i += 1
            self.ind += 1
            visualization(self)
            
        # return rrt_tree
        self.done = True
        self.Path, _ = path(self)
        visualization(self)
        plt.show()
        

    def Nearest(self, tree, target):
        # TODO use kdTree to speed up search
        return nearest(self, target, isset=True)

    def Extend(self, env, nearest, target):
        extended, dist = steer(self, nearest, target, DIST = True)
        collide, _ = isCollide(self, nearest, target, dist)
        return extended, collide

    def AddNode(self, tree, nearest, extended):
        # TODO use a kdtree
        self.V.append(extended)
        self.Parent[extended] = nearest

    def RandomState(self):
        # generate a random, obstacle free state
        xrand = sampleFree(self, bias=0)
        return xrand

    #--------- insight to the rrt_extend
    def ChooseTarget(self, state):
        # return the goal, or randomly choose a state in the waypoints based on probs
        p = np.random.uniform()
        if len(self.V) == 1:
            i = 0
        else:
            i = np.random.randint(0, high = len(self.V) - 1)
        if 0 < p < self.GoalProb:
            return self.xt
        elif self.GoalProb < p < self.GoalProb + self.WayPointProb:
            return self.V[i]
        elif self.GoalProb + self.WayPointProb < p < 1:
            return tuple(self.RandomState())
        
if __name__ == '__main__':
    t = extend_rrt()
    _ = t.RRTplan(t.env, t.x0, t.xt)