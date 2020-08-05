"""
This is fast marching tree* code for 3D
@author: yue qi 
source: Janson, Lucas, et al. "Fast marching tree: A fast marching sampling-based method 
        for optimal motion planning in many dimensions." 
        The International journal of robotics research 34.7 (2015): 883-921.
"""
import numpy as np
import matplotlib.pyplot as plt
import time
import copy


import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide

class FMT_star:

    def __init__(self):
        self.env = env()
        # note that the xgoal could be a region since this algorithm is a multiquery method
        self.xinit, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.n = 100 # number of samples
        # sets
        self.V = self.generateSampleSet(self.n - 2) # set of all nodes
        self.Vopen = set(self.xinit) # open set
        self.Vclosed = set() # closed set
        self.Vunvisited = copy.deepcopy(self.V) # unvisited set
        self.Vunvisited.add(self.xgoal)
        # cost to come
        self.c = {}

    def generateSampleSet(self, n):
        V = set()
        for i in range(n):
            V.add(sampleFree(self, bias = 0.0))
        return V

    def Near(self, nodeset, node, range):
        newSet = set()
        return newSet

    def Path(self, T):
        V, E = T
        path = []
        return path

    def Cost(self, x, y):
        pass

    def FMTrun(self):
        z = copy.deepcopy(self.xinit)
        Nz = self.Near(self.Vunvisited, z, rn)
        E = set()
        # Save(Nz, z)
        while z != self.xgoal:
            Vopen_new = set()
            Xnear = Nz.intersection(self.Vunvisited)
            for x in Xnear:
                Nx = self.Near(self.V.difference(set(x)), x, rn)
                # Save(Nx, x)
                Ynear = Nx.intersection(self.Vopen)
                ymin = Ynear[np.argmin([self.c[y] + self.Cost(y,x) for y in Ynear])] # DP programming equation
                collide, _ = self.isCollide(ymin, x)
                if not collide:
                    E = E.add((ymin, x)) # straight line joining ymin and x is collision free
                    Vopen_new.add(x)
                    self.Vunvisited = self.Vunvisited.difference(set(x))
                    self.c[x] = self.c[ymin] + self.Cost(ymin, x) # cost-to-arrive from xinit in tree T = (VopenUVclosed, E)
            self.Vopen = (self.Vopen.union(Vopen_new)).difference(set(z))
            self.Vclosed = self.Vclosed.union(set(z))
            if len(self.Vopen) > 0:
                return 'Failure'
            z = np.argmin([self.c[y] for y in self.Vopen])
        return self.Path(z, T = (self.Vopen.union(self.Vclosed), E))



