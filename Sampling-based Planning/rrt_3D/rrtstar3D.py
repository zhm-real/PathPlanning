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

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling-based Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path, edgeset, hash3D, dehash


class rrtstar():
    def __init__(self):
        self.env = env()
        self.Parent = {}
        self.E = edgeset()
        self.V = []
        self.i = 0
        self.maxiter = 10000 # at least 4000 in this env
        self.stepsize = 0.5
        self.gamma = 500
        self.eta = 1.1*self.stepsize
        self.Path = []
        self.done = False

    def wireup(self,x,y):
        self.E.add_edge([x,y]) # add edge
        self.Parent[hash3D(x)] = y

    def removewire(self,xnear):
        xparent = self.Parent[hash3D(xnear)]
        a = [xnear,xparent]
        self.E.remove_edge(a) # remove and replace old the connection

    def reached(self):
        self.done = True
        xn = near(self,self.env.goal)
        c = [cost(self,x) for x in xn]
        xncmin = xn[np.argmin(c)]
        self.wireup(self.env.goal,xncmin)
        self.V.append(self.env.goal)
        self.Path,self.D = path(self)

    def run(self):
        self.V.append(self.env.start)
        self.ind = 0
        xnew = self.env.start
        print('start rrt*... ')
        self.fig = plt.figure(figsize = (10,8))
        while self.ind < self.maxiter:
            xrand    = sampleFree(self)
            xnearest = nearest(self,xrand)
            xnew     = steer(self,xnearest,xrand)
            if not isCollide(self,xnearest,xnew):
                Xnear = near(self,xnew)
                self.V.append(xnew) # add point
                # visualization(self)
                # minimal path and minimal cost
                xmin, cmin = xnearest, cost(self, xnearest) + getDist(xnearest, xnew)
                # connecting along minimal cost path
                for xnear in Xnear:
                    c1 = cost(self, xnear) + getDist(xnew, xnear)
                    if not isCollide(self, xnew, xnear) and c1 < cmin:
                        xmin, cmin = xnear, c1
                self.wireup(xnew, xmin)
                # rewire
                for xnear in Xnear:
                    c2 = cost(self, xnew) + getDist(xnew, xnear)
                    if not isCollide(self, xnew, xnear) and c2 < cost(self, xnear):
                        self.removewire(xnear)
                        self.wireup(xnear, xnew)
                self.i += 1
            self.ind += 1
        # max sample reached
        self.reached()
        print('time used = ' + str(time.time()-starttime))
        print('Total distance = '+str(self.D))
        visualization(self)
        plt.show()
        

if __name__ == '__main__':
    p = rrtstar()
    starttime = time.time()
    p.run()
    
