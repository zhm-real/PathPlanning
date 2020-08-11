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
import time
import copy


import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide
from rrt_3D.plot_util3D import make_get_proj, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent
from rrt_3D.queue import MinheapPQ

class BIT_star:

    def __init__(self):
        self.env = env()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.maxiter = 1000 # used for determining how many batches needed
        # radius calc
        self.eta = 1 # bigger or equal to 1
        self.n = 1000
        self.nn = 1 # TODO
        
        self.edgeCost = {} # corresponding to c
        self.heuristic_edgeCost = {} # correspoinding to c_hat

    def run(self):
        self.V = {self.xstart}
        self.E = set()
        self.Parent = {}
        self.T = (self.V, self.E) # tree
        self.Xsamples = {self.xgoal}
        self.QE = set()
        self.QV = set()
        self.r = np.inf
        ind = 0
        while True:
            # for the first round
            if len(self.QE) == 0 and len(self.QV) == 0:
                self.Prune(self.g_T(self.xgoal))
                self.Xsamples = self.Sample(m, self.g_T(self.xgoal)) # sample function
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
            ind += 1
            if ind > self.maxiter:
                break
            return self.T

    def Sample(self, m, cost):
        # TODO need the informed rrt
        pass
    
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
        return 2 * self.eta * (1 + 1/self.n) ** (1/self.n) * \
            (self.Lambda(self.Xf_hat(self.V)) / self.Zeta ) ** (1/self.n) * \
            (np.log(q) / q) ** (1/self.n)

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
         
    