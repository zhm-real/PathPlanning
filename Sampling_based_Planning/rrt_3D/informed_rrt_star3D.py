# informed RRT star in 3D
"""
This is IRRT* code for 3D
@author: yue qi 
source: J. D. Gammell, S. S. Srinivasa, and T. D. Barfoot, “Informed RRT*:
        Optimal sampling-based path planning focused via direct sampling of
        an admissible ellipsoidal heuristic,” in IROS, 2997–3004, 2014.
"""
import numpy as np
import matplotlib.pyplot as plt
import time
import copy


import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, isinside, near, nearest
from rrt_3D.plot_util3D import make_get_proj, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent
from rrt_3D.queue import MinheapPQ

class IRRT:

    def __init__(self):
        self.env = env()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.Parent = {}
        self.N = 10000 # used for determining how many batches needed
        self.ind = 0
        self.i = 0
        # rrt* near 
        self.stepsize = 0.5
        self.gamma = 500
        self.eta = self.stepsize
        self.rgoal = self.stepsize

        self.done = False

    def Informed_rrt(self):
        self.V = [self.xstart]
        self.E = set()
        self.Xsoln = set()
        self.T = (self.V, self.E)
        
        c = 1
        while self.ind <= self.N:
            print(self.ind)
            # print(self.i)
            if len(self.Xsoln) == 0:
                cbest = np.inf
            else:
                cbest = min({self.cost(xsln) for xsln in self.Xsoln})
            xrand = self.Sample(self.xstart, self.xgoal, cbest)
            xnearest = nearest(self, xrand)
            xnew, dist = steer(self, xnearest, xrand)
            # print(xnew)
            collide, _ = isCollide(self, xnearest, xnew, dist=dist)
            if not collide:
                self.V.append(xnew)
                Xnear = near(self, xnew)
                xmin = xnearest
                cmin = self.cost(xmin) + c * self.line(xnearest, xnew)
                for xnear in Xnear:
                    xnear = tuple(xnear)
                    cnew = self.cost(xnear) + c * self.line(xnear, xnew)
                    if cnew < cmin:
                        collide, _ = isCollide(self, xnear, xnew)
                        if not collide:
                            xmin = xnear
                            cmin = cnew
                self.E.add((xmin, xnew))
                self.Parent[xnew] = xmin
                
                for xnear in Xnear:
                    xnear = tuple(xnear)
                    cnear = self.cost(xnear)
                    cnew = self.cost(xnew) + c * self.line(xnew, xnear)
                    # rewire
                    if cnew < cnear:
                        collide, _ = isCollide(self, xnew, xnear)
                        if not collide:
                            xparent = self.Parent[xnear]
                            self.E.difference_update((xparent, xnear))
                            self.E.add((xnew, xnear))
                            self.Parent[xnear] = xnew
                self.i += 1
                if self.InGoalRegion(xnew):
                    print('reached')
                    self.Xsoln.add(xnew)
                
            self.ind += 1
        # return tree
        return self.T
                
    def Sample(self, xstart, xgoal, cmax):
        # sample within a eclipse 
        if cmax < np.inf:
            cmin = getDist(xgoal, xstart)
            xcenter = np.array([(xgoal[0] + xstart[0]) / 2, (xgoal[1] + xstart[1]) / 2, (xgoal[2] + xstart[2]) / 2])
            C = self.RotationToWorldFrame(xstart, xgoal)
            r = np.zeros(3)
            r[0] = cmax /2
            for i in range(1,3):
                r[i] = np.sqrt(cmax**2 - cmin**2) / 2
            L = np.diag(r) # R3*3 
            xball = self.SampleUnitBall() # np.array
            x =  C@L@xball + xcenter
            if not isinside(self, x): # intersection with the state space
                xrand = x
            else:
                return self.Sample(xstart, xgoal, cmax)
        else:
            xrand = sampleFree(self, bias = 0.0)
        return xrand

    def SampleUnitBall(self):
        # uniform sampling in spherical coordinate system in 3D
        # sample radius
        r = np.random.uniform(0.0, 1.0)
        theta = np.random.uniform(0, np.pi)
        phi = np.random.uniform(0, 2 * np.pi)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return np.array([x,y,z])

    def RotationToWorldFrame(self, xstart, xgoal):
        # S0(n): such that the xstart and xgoal are the center points
        d = getDist(xstart, xgoal)
        xstart, xgoal = np.array(xstart), np.array(xgoal)
        a1 = (xgoal - xstart) / d
        M = np.outer(a1,[1,0,0])
        U, S, V = np.linalg.svd(M)
        C = U@np.diag([1, 1, np.linalg.det(U)*np.linalg.det(V)])@V.T
        return C

    def InGoalRegion(self, x):
        # Xgoal = {x in Xfree | \\x-xgoal\\2 <= rgoal}
        return getDist(x, self.xgoal) <= self.rgoal

    def cost(self, x):
        # actual cost 
        '''here use the additive recursive cost function'''
        if x == self.xstart:
            return 0.0
        if x not in self.Parent:
            return np.inf
        return self.cost(self.Parent[x]) + getDist(x, self.Parent[x])

    def line(self, x, y):
        return getDist(x, y)

    def g_hat(self, x):
        # heuristic estimate from start to x
        return getDist(x, self.xstart)

    def h_hat(self, x):
        # heuristic estimate from x to goal
        return getDist(x, self.xgoal)

if __name__ == '__main__':
    A = IRRT()
    A.Informed_rrt()
