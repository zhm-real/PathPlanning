"""
This is rrt star code for 3D
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


class rrt():
    def __init__(self):
        self.env = env()
        self.Parent = {}
        self.V = []
        # self.E = edgeset()
        self.i = 0
        self.maxiter = 10000
        self.stepsize = 0.5
        self.Path = []
        self.done = False
        self.x0 = tuple(self.env.start)
        self.xt = tuple(self.env.goal)

        
        self.ind = 0
        # self.fig = plt.figure(figsize=(10, 8))

    def wireup(self, x, y):
        # self.E.add_edge([s, y])  # add edge
        self.Parent[x] = y

    def run(self):
        self.V.append(self.x0)
        while self.ind < self.maxiter:
            xrand = sampleFree(self)
            xnearest = nearest(self, xrand)
            xnew, dist = steer(self, xnearest, xrand)
            collide, _ = isCollide(self, xnearest, xnew, dist=dist)
            if not collide:
                self.V.append(xnew)  # add point
                self.wireup(xnew, xnearest)

                if getDist(xnew, self.xt) <= self.stepsize:
                    self.wireup(self.xt, xnew)
                    self.Path, D = path(self)
                    print('Total distance = ' + str(D))
                    break
                visualization(self)
                self.i += 1
            self.ind += 1
            # if the goal is really reached
            
        self.done = True
        visualization(self)
        plt.show()


if __name__ == '__main__':
    p = rrt()
    starttime = time.time()
    p.run()
    print('time used = ' + str(time.time() - starttime))
