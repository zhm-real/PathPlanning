# This is Batched Informed Tree star 3D algorithm 
# implementation
"""
This is ABIT* code for 3D
@author: yue qi 
Algorithm 1
source: Gammell, Jonathan D., Siddhartha S. Srinivasa, and Timothy D. Barfoot. "Batch informed trees (BIT*): 
        Sampling-based optimal planning via the heuristically guided search of implicit random geometric graphs." 
        2015 IEEE international conference on robotics and automation (ICRA). IEEE, 2015.  
and 
source: Gammell, Jonathan D., Timothy D. Barfoot, and Siddhartha S. Srinivasa. 
        "Batch Informed Trees (BIT*): Informed asymptotically optimal anytime search." 
        The International Journal of Robotics Research 39.5 (2020): 543-567.
"""

import numpy as np
import matplotlib.pyplot as plt
from numpy.matlib import repmat
import time
import copy


import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, isinside, isinbound
from rrt_3D.plot_util3D import set_axes_equal, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent
from rrt_3D.queue import MinheapPQ

#---------methods to draw ellipse during sampling
def CreateUnitSphere(r = 1):
    phi = np.linspace(0,2*np.pi, 256).reshape(256, 1) # the angle of the projection in the xy-plane
    theta = np.linspace(0, np.pi, 256).reshape(-1, 256) # the angle from the polar axis, ie the polar angle
    radius = r

    # Transformation formulae for a spherical coordinate system.
    x = radius*np.sin(theta)*np.cos(phi)
    y = radius*np.sin(theta)*np.sin(phi)
    z = radius*np.cos(theta)
    return (x, y, z)

def draw_ellipsoid(ax, C, L, xcenter):
    (xs, ys, zs) = CreateUnitSphere()
    pts = np.array([xs, ys, zs])
    pts_in_world_frame = C@L@pts + xcenter
    ax.plot_surface(pts_in_world_frame[0], pts_in_world_frame[1], pts_in_world_frame[2], alpha=0.05, color="g")

class BIT_star:
# ---------initialize and run
    def __init__(self, show_ellipse=False):
        self.env = env()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.maxiter = 1000 # used for determining how many batches needed
        
        # radius calc parameter:
        # larger value makes better 1-time-performance, but longer time trade off
        self.eta = 7 # bigger or equal to 1

        # sampling 
        self.m = 400 # number of samples for one time sample
        self.d = 3 # dimension we work with
        
        # instance of the cost to come gT
        self.g = {self.xstart:0, self.xgoal:np.inf}

        # draw ellipse
        self.show_ellipse = show_ellipse

        # denote if the path is found 
        self.done = False
        self.Path = []
        
        # for drawing the ellipse
        self.C = np.zeros([3,3])
        self.L = np.zeros([3,3])
        self.xcenter = np.zeros(3)
        self.show_ellipse = show_ellipse

    def run(self):
        self.V = {self.xstart} # node expanded
        self.E = set() # edge set
        self.Parent = {} # Parent relation
        # self.T = (self.V, self.E) # tree
        self.Xsamples = {self.xgoal} # sampled set
        self.QE = set() # edges in queue
        self.QV = set() # nodes in queue
        self.r = np.inf # radius for evaluation
        self.ind = 0
        num_resample = 0
        while True:
            # for the first round
            print('round '+str(self.ind))
            self.visualization()
            # print(len(self.V))
            if len(self.QE) == 0 and len(self.QV) == 0:
                self.Prune(self.g_T(self.xgoal))
                self.Xsamples = self.Sample(self.m, self.g_T(self.xgoal)) # sample function
                self.Xsamples.add(self.xgoal) # adding goal into the sample
                self.Vold = {v for v in self.V}
                self.QV = {v for v in self.V}
                # setting the radius 
                if self.done:
                    self.r = 2 # sometimes the original radius criteria makes the radius too small to improve existing tree
                    num_resample += 1
                else:
                    self.r = self.radius(len(self.V) + len(self.Xsamples)) # radius determined with the sample size and dimension of conf space
            while self.BestQueueValue(self.QV, mode = 'QV') <= self.BestQueueValue(self.QE, mode = 'QE'):
                self.ExpandVertex(self.BestInQueue(self.QV, mode = 'QV'))
            (vm, xm) = self.BestInQueue(self.QE, mode = 'QE')
            self.QE.remove((vm, xm))
            if self.g_T(vm) + self.c_hat(vm, xm) + self.h_hat(xm) < self.g_T(self.xgoal):
                cost = self.c(vm, xm)
                if self.g_hat(vm) + cost + self.h_hat(xm) < self.g_T(self.xgoal):
                    if self.g_T(vm) + cost < self.g_T(xm):
                        if xm in self.V:
                            self.E.difference_update({(v, x) for (v, x) in self.E if x == xm})
                        else:
                            self.Xsamples.remove(xm)
                            self.V.add(xm)
                            self.QV.add(xm)
                        self.g[xm] = self.g[vm] + cost
                        self.E.add((vm, xm))
                        self.Parent[xm] = vm # add parent or update parent
                        self.QE.difference_update({(v, x) for (v, x) in self.QE if x == xm and (self.g_T(v) + self.c_hat(v, xm)) >= self.g_T(xm)})
            
            # reinitializing sampling
            else:
                self.QE = set()
                self.QV = set()
            self.ind += 1
            
            # if the goal is reached
            if self.xgoal in self.Parent:
                print('locating path...')
                self.done = True
                self.Path = self.path()

            # if the iteration is bigger
            if self.ind > self.maxiter:
                break

        print('complete')
        print('number of times resampling ' + str(num_resample))

# ---------IRRT utils
    def Sample(self, m, cmax, bias = 0.05, xrand = set()):
        # sample within a eclipse 
        print('new sample')
        if cmax < np.inf:
            cmin = getDist(self.xgoal, self.xstart)
            xcenter = np.array([(self.xgoal[0] + self.xstart[0]) / 2, (self.xgoal[1] + self.xstart[1]) / 2, (self.xgoal[2] + self.xstart[2]) / 2])
            C = self.RotationToWorldFrame(self.xstart, self.xgoal)
            r = np.zeros(3)
            r[0] = cmax /2
            for i in range(1,3):
                r[i] = np.sqrt(cmax**2 - cmin**2) / 2
            L = np.diag(r) # R3*3 
            xball = self.SampleUnitBall(m) # np.array
            x =  (C@L@xball).T + repmat(xcenter, len(xball.T), 1)
            # x2 = set(map(tuple, x))
            self.C = C # save to global var
            self.xcenter = xcenter
            self.L = L
            x2 = set(map(tuple, x[np.array([not isinside(self, state) and isinbound(self.env.boundary, state) for state in x])])) # intersection with the state space
            xrand.update(x2)
            # if there are samples inside obstacle: recursion
            if len(x2) < m:
                return self.Sample(m - len(x2), cmax, bias=bias, xrand=xrand)
        else:
            for i in range(m):
                xrand.add(tuple(sampleFree(self, bias = bias)))
        return xrand

    def SampleUnitBall(self, n):
        # uniform sampling in spherical coordinate system in 3D
        # sample radius
        r = np.random.uniform(0.0, 1.0, size = n)
        theta = np.random.uniform(0, np.pi, size = n)
        phi = np.random.uniform(0, 2 * np.pi, size = n)
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

#----------BIT_star particular
    def ExpandVertex(self, v):
        self.QV.remove(v)
        Xnear = {x for x in self.Xsamples if getDist(x, v) <= self.r}
        self.QE.update({(v, x) for x in Xnear if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < self.g_T(self.xgoal)})
        if v not in self.Vold:
            Vnear = {w for w in self.V if getDist(w, v) <= self.r}
            self.QE.update({(v,w) for w in Vnear if \
                ((v,w) not in self.E) and \
                (self.g_hat(v) + self.c_hat(v, w) + self.h_hat(w) < self.g_T(self.xgoal)) and \
                (self.g_T(v) + self.c_hat(v, w) < self.g_T(w))})

    def Prune(self, c):
        self.Xsamples = {x for x in self.Xsamples if self.f_hat(x) >= c}
        self.V.difference_update({v for v in self.V if self.f_hat(v) >= c})
        self.E.difference_update({(v, w) for (v, w) in self.E if (self.f_hat(v) > c) or (self.f_hat(w) > c)})
        self.Xsamples.update({v for v in self.V if self.g_T(v) == np.inf})
        self.V.difference_update({v for v in self.V if self.g_T(v) == np.inf})

    def radius(self, q):
        return 2 * self.eta * (1 + 1/self.d) ** (1/self.d) * \
            (self.Lambda(self.Xf_hat(self.V)) / self.Zeta() ) ** (1/self.d) * \
            (np.log(q) / q) ** (1/self.d)

    def Lambda(self, inputset):
        # lebesgue measure of a set, defined as 
        # mu: L(Rn) --> [0, inf], e.g. volume
        return len(inputset)

    def Xf_hat(self, X):
        # the X is a set, defined as {x in X | fhat(x) <= cbest}
        # where cbest is current best cost.
        cbest = self.g_T(self.xgoal)
        return {x for x in X if self.f_hat(x) <= cbest}

    def Zeta(self):
        # Lebesgue measure of a n dimensional unit ball
        # since it's the 3D, use volume
        return 4/3 * np.pi

    def BestInQueue(self, inputset, mode):
        # returns the best vertex in the vertex queue given this ordering
        # mode = 'QE' or 'QV'
        if mode == 'QV':
            V = {state: self.g_T(state) + self.h_hat(state) for state in self.QV}
        if mode == 'QE':
            V = {state: self.g_T(state[0]) + self.c_hat(state[0], state[1]) + self.h_hat(state[1]) for state in self.QE}
        if len(V) == 0:
            print(mode + 'empty')
            return None
        return min(V, key = V.get)

    def BestQueueValue(self, inputset, mode):
        # returns the best value in the vertex queue given this ordering
        # mode = 'QE' or 'QV'
        if mode == 'QV':
            V = {self.g_T(state) + self.h_hat(state) for state in self.QV}
        if mode == 'QE':
            V = {self.g_T(state[0]) + self.c_hat(state[0], state[1]) + self.h_hat(state[1]) for state in self.QE}
        if len(V) == 0:
            return np.inf
        return min(V)

    def g_hat(self, v):
        return getDist(self.xstart, v)

    def h_hat(self, v):
        return getDist(self.xgoal, v)

    def f_hat(self, v):
        # f = g + h: estimate cost
        return self.g_hat(v) + self.h_hat(v)

    def c(self, v, w):
        # admissible estimate of the cost of an edge between state v, w
        collide, dist = isCollide(self, v, w)
        if collide:
            return np.inf
        else: 
            return dist

    def c_hat(self, v, w):
        # c_hat < c < np.inf
        # heuristic estimate of the edge cost, since c is expensive
        return getDist(v, w)

    def g_T(self, v):
        # represent cost-to-come from the start in the tree, 
        # if the state is not in tree, or unreachable, return inf
        if v not in self.g:
            self.g[v] = np.inf
        return self.g[v]

    def path(self):
        path = []
        s = self.xgoal
        i = 0
        while s != self.xstart:
            path.append((s, self.Parent[s]))
            s = self.Parent[s]
            if i > self.m:
                break
            i += 1
        return path
         
    def visualization(self):
        if self.ind % 20 == 0:
            V = np.array(list(self.V))
            Xsample = np.array(list(self.Xsamples))
            edges = list(map(list, self.E))
            Path = np.array(self.Path)
            start = self.env.start
            goal = self.env.goal
            # edges = E.get_edge()
            #----------- list structure
            # edges = []
            # for i in self.Parent:
            #     edges.append([i,self.Parent[i]])
            #----------- end
            # generate axis objects
            ax = plt.subplot(111, projection='3d')
            
            # ax.view_init(elev=0.+ 0.03*self.ind/(2*np.pi), azim=90 + 0.03*self.ind/(2*np.pi))
            # ax.view_init(elev=0., azim=90.)
            ax.view_init(elev=90., azim=60.)
            # ax.view_init(elev=-8., azim=180)
            ax.clear()
            # drawing objects
            draw_Spheres(ax, self.env.balls)
            draw_block_list(ax, self.env.blocks)
            if self.env.OBB is not None:
                draw_obb(ax, self.env.OBB)
            draw_block_list(ax, np.array([self.env.boundary]), alpha=0)
            draw_line(ax, edges, visibility=0.75, color='g')
            draw_line(ax, Path, color='r')
            if self.show_ellipse:
                draw_ellipsoid(ax, self.C, self.L, self.xcenter) # beware, depending on start and goal position, this might be bad for vis
            if len(V) > 0:
                ax.scatter3D(V[:, 0], V[:, 1], V[:, 2], s=2, color='g', )
            if len(Xsample) > 0: # plot the sampled points
                ax.scatter3D(Xsample[:, 0], Xsample[:, 1], Xsample[:, 2], s=1, color='b',)
            ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
            ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k')
            # adjust the aspect ratio
            ax.dist = 11
            set_axes_equal(ax)
            make_transparent(ax)
            #plt.xlabel('s')
            #plt.ylabel('y')
            ax.set_axis_off()
            plt.pause(0.0001)


if __name__ == '__main__':
    Newprocess = BIT_star(show_ellipse=False)
    Newprocess.run()