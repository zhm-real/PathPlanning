
"""
This is rrt star code for 3D
@author: yue qi
"""
import numpy as np
from numpy.matlib import repmat
from collections import defaultdict
import time

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__))+"/../../Sampling-based Planning/")

from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path, edgeset



class rrtstar():
    def __init__(self):
        self.env = env()
        self.Parent = defaultdict(lambda: defaultdict(dict))
        self.V = []
        self.E = edgeset()
        self.i = 0
        self.maxiter = 10000
        self.stepsize = 0.5
        self.Path = []

    def wireup(self,x,y):
        self.E.add_edge([x,y]) # add edge
        self.Parent[str(x[0])][str(x[1])][str(x[2])] = y

    def run(self):
        self.V.append(self.env.start)
        ind = 0
        xnew = self.env.start
        while ind < self.maxiter and getDist(xnew,self.env.goal) > 1:
            xrand    = sampleFree(self)
            xnearest = nearest(self,xrand)
            xnew     = steer(self,xnearest,xrand)
            if not isCollide(self,xnearest,xnew):
                self.V.append(xnew) # add point
                self.wireup(xnew,xnearest)
                #visualization(self)
                self.i += 1
            ind += 1
            if getDist(xnew,self.env.goal) <= 1:
                self.wireup(self.env.goal,xnew)
                self.Path,D = path(self)
                print('Total distance = '+str(D))
        visualization(self)

if __name__ == '__main__':
    p = rrtstar()
    starttime = time.time()
    p.run()
    print('time used = ' + str(time.time()-starttime))