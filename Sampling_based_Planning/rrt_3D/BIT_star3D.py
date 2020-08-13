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
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, isinside
from rrt_3D.plot_util3D import make_get_proj, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent
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
        self.maxiter = 3000 # used for determining how many batches needed
        # radius calc
        self.eta = 20 # bigger or equal to 1
        self.m = 1000 # number of samples for one time sample
        self.d = 3 # dimension we work with
        self.Path = []
        
        self.edgeCost = {} # corresponding to c
        self.heuristic_edgeCost = {} # correspoinding to c_hat

        # draw ellipse
        self.show_ellipse = show_ellipse

    def run(self):
        self.V = {self.xstart}
        self.E = set()
        self.Parent = {}
        self.T = (self.V, self.E) # tree
        self.Xsamples = {self.xgoal}
        self.QE = set()
        self.QV = set()
        self.r = np.inf
        self.ind = 0
        while True:
            # for the first round
            print(self.ind)
            print(self.r)
            self.visualization()
            # print(len(self.V))
            if len(self.QE) == 0 and len(self.QV) == 0:
                self.Prune(self.g_T(self.xgoal))
                self.Xsamples = self.Sample(self.m, self.g_T(self.xgoal)) # sample function
                self.Vold = copy.deepcopy(self.V)
                self.QV = copy.deepcopy(self.V)
                self.r = self.radius(len(self.V) + len(self.Xsamples))
            while self.BestQueueValue(self.QV, mode = 'QV') <= self.BestQueueValue(self.QE, mode = 'QE'):
                self.ExpandVertex(self.BestInQueue(self.QV, mode = 'QV'))
            (vm, xm) = self.BestInQueue(self.QE, mode = 'QE')
            self.QE.difference_update({(vm, xm)})
            if self.g_T(vm) + self.c_hat(vm, xm) + self.h_hat(xm) < self.g_T(self.xgoal):
                if self.g_hat(vm) + self.c(vm, xm) + self.h_hat(xm) < self.g_T(self.xgoal):
                    if self.g_T(vm) + self.c(vm, xm) < self.g_T(xm):
                        if xm in self.V:
                            self.E.difference_update({(v, x) for (v, x) in self.E if x == xm})
                        else:
                            self.Xsamples.difference_update({xm})
                            self.V.add(xm)
                            self.QV.add(xm)
                        self.E.add((vm, xm))
                        self.Parent[vm] = xm # add parent or update parent
                        self.QE.difference_update({(v, x) for (v, x) in self.QE if x == xm and self.g_T(v) + self.c_hat(v, x) >= self.g_T(x)})

            else:
                self.QE = set()
                self.QV = set()
            self.ind += 1

            if self.ind > self.maxiter:
                break
        return self.T

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
            x2 = set(map(tuple, x[np.array([not isinside(self, state) for state in x])])) # intersection with the state space
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
        self.QV.difference_update({v})
        Xnear = {x for x in self.Xsamples if getDist(x, v) <= self.r}
        self.QE.update({(v, x) for v in self.V for x in Xnear if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < self.g_T(self.xgoal)})
        if v not in self.Vold:
            Vnear = {w for w in self.V if getDist(w, v) <= self.r}
            self.QE.update({(v,w) for v in self.V for w in Vnear if \
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
        _, best_state = self.find_best(inputset, mode)
        return best_state

    def BestQueueValue(self, inputset, mode):
        # returns the best value in the vertex queue given this ordering
        # mode = 'QE' or 'QV'
        best_val, _ = self.find_best(inputset, mode)
        return best_val

    def find_best(self, inputset, mode):
        min_val, min_state = np.inf, None
        for state in inputset:
            if mode == 'QE':
                curr_val = self.g_T(state[0]) + self.c_hat(state[0], state[1]) + self.h_hat(state[1])
            elif mode == 'QV':
                curr_val = self.g_T(state) + self.h_hat(state)
            if curr_val < min_val:
                min_val, min_state = curr_val, state
        return min_val, min_state
    
    def g_hat(self, v):
        return getDist(self.xstart, v)

    def h_hat(self, v):
        return getDist(self.xgoal, v)

    def f_hat(self, v):
        # f = g + h: estimate cost
        return self.g_hat(v) + self.h_hat(v)

    def c(self, v, w):
        # admissible estimate of the cost of an edge between state v, w
        if (v,w) in self.edgeCost:
            pass
        else:
            collide, dist = isCollide(self, v, w)
            if collide:
                self.edgeCost[(v,w)] = np.inf
            else: 
                self.edgeCost[(v,w)] = dist
        return self.edgeCost[(v,w)]

    def c_hat(self, v, w):
        # c_hat < c < np.inf
        # heuristic estimate of the edge cost, since c is expensive
        if (v,w) in self.heuristic_edgeCost:
            pass
        else:
            self.heuristic_edgeCost[(v,w)] = getDist(v, w)
        return self.heuristic_edgeCost[(v,w)]

    def g_T(self, v):
        # represent cost-to-come from the start in the tree, 
        # if the state is not in tree, or unreachable, return inf
        if v in self.Parent:
            cost_to_come = 0
            while v != self.xstart:
                cost_to_come += self.c(v, self.Parent[v])
                v = self.Parent[v]
            return cost_to_come
        elif v == self.xstart:
            return 0
        else:
            return np.inf
         
    def visualization(self):
        if self.ind % 20 == 0:
            V = np.array(list(self.V))
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
            ax.view_init(elev=8., azim=90.)
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
            ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
            ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k')
            # adjust the aspect ratio
            xmin, xmax = self.env.boundary[0], self.env.boundary[3]
            ymin, ymax = self.env.boundary[1], self.env.boundary[4]
            zmin, zmax = self.env.boundary[2], self.env.boundary[5]
            dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin
            ax.get_proj = make_get_proj(ax, 1 * dx, 1 * dy, 2 * dy)
            make_transparent(ax)
            #plt.xlabel('s')
            #plt.ylabel('y')
            ax.set_axis_off()
            plt.pause(0.0001)


if __name__ == '__main__':
    Newprocess = BIT_star()
    Newprocess.run()
    # Xsamples = Newprocess.Sample(1000, 140)
    # print(len(Xsamples))