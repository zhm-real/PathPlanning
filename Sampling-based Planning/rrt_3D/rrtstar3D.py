
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
        self.E = edgeset()
        self.V = []
        self.i = 0
        self.maxiter = 10000
        self.stepsize = 0.5
        self.Path = []

    def wireup(self,x,y):
        self.E.add_edge([x,y]) # add edge
        self.Parent[str(x[0])][str(x[1])][str(x[2])] = y

    def removewire(self,xnear):
        xparent = self.Parent[str(xnear[0])][str(xnear[1])][str(xnear[2])]
        a = [xnear,xparent]
        self.E.remove_edge(a) # remove and replace old the connection

    def run(self):
        self.V.append(self.env.start)
        ind = 0
        xnew = self.env.start
        while ind < self.maxiter and getDist(xnew,self.env.goal) > 1:
        #while ind < self.maxiter:
            xrand    = sampleFree(self)
            xnearest = nearest(self,xrand)
            xnew     = steer(self,xnearest,xrand)
            if not isCollide(self,xnearest,xnew):
                Xnear = near(self,xnew)
                self.V.append(xnew) # add point
                # visualization(self)
                # minimal path and minimal cost
                xmin,cmin = xnearest,cost(self,xnearest) + getDist(xnearest,xnew)
                # connecting along minimal cost path
                if self.i == 0:
                    c1 = cost(self,Xnear) + getDist(xnew,Xnear)
                    if not isCollide(self,xnew,Xnear) and c1 < cmin:
                            xmin,cmin = Xnear,c1
                    self.wireup(xnew,xmin)
                else:
                    for xnear in Xnear:
                        c1 = cost(self,xnear) + getDist(xnew,xnear)
                        if not isCollide(self,xnew,xnear) and c1 < cmin:
                            xmin,cmin = xnear,c1
                    self.wireup(xnew,xmin)
                    # rewire
                    for xnear in Xnear:
                        c2 = cost(self,xnew) + getDist(xnew,xnear)
                        if not isCollide(self,xnew,xnear) and c2 < cost(self,xnear):
                            self.removewire(xnear)
                            self.wireup(xnear,xnew)
                self.i += 1
            ind += 1
            # when the goal is reached
            if getDist(xnew,self.env.goal) <= 1:
                self.wireup(self.env.goal,xnew)
                self.Path,self.D = path(self)
        visualization(self)
        print('Total distance = '+str(self.D))

if __name__ == '__main__':
    p = rrtstar()
    starttime = time.time()
    p.run()
    print('time used = ' + str(time.time()-starttime))