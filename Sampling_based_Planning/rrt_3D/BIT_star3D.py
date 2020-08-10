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
        self.maxiter = 1000
        # radius calc
        self.eta = 1 # bigger or equal to 1
        self.n = 1000
        self.Xf_hat = 1 # TODO
        self.nn = 1 # TODO

    def run(self):
        V = {self.xstart}
        E = set()
        T = (V, E) # tree
        Xsamples = {self.xgoal}
        QE = set()
        QV = set()
        r = np.inf
        ind = 0
        while True:
            if len(QE) == 0 and len(QV) == 0:
                Xsamples, V, E = self.Prune(self.g_T(self.xgoal), Xsamples, V, E)
                Vold = copy.deepcopy(V)
                QV = copy.deepcopy(V)
                r = self.radius(len(V) + len(Xsamples))
            while self.BestQueueValue(QV) <= self.BestQueueValue(QE):
                QV, QE = self.ExpandVertex(self.BestInQueue(QV), QV, QE, Xsamples, Vold, E, V, r)
            (vm, xm) = self.BestInQueue(QE)
            QE.difference_update({(vm, xm)})
            if self.g_T(vm) + self.c_hat(vm, xm) + self.h_hat(xm) < self.g_T(self.xgoal):
                if self.g_hat(vm) + self.c(vm, xm) + self.h_hat(xm) < self.g_T(self.xgoal):
                    if self.g_T(vm) + self.c(vm, xm) < self.g_T(xm):
                        if xm in V:
                            E.difference_update({(v, x) for (v, x) in E if x == xm})
                        else:
                            Xsamples.difference_update({xm})
                            V.add(xm)
                            QV.add(xm)
                        E.add((vm, xm))
                        QE.difference_update({(v, x) for (v, x) in QE if x == xm and self.g_T(v) + self.c_hat(v, x) >= self.g_T(x)})

            else:
                QE = set()
                QV = set()
            ind += 1
            if ind > self.maxiter:
                break
            return T

    def ExpandVertex(self, v , QV, QE, Xsamples, Vold, E, V, r):
        QV.difference_update({v})
        Xnear = {x for x in Xsamples if getDist(x, v) <= r}
        QE = {(v, x) for v in V for x in Xnear if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < self.g_T(self.xgoal)}
        if v not in Vold:
            Vnear = {w for w in V if getDist(w, v) <= r}
            QE.update({(v,w) for v in V for w in Vnear if \
                ((v,w) not in E) and \
                (self.g_hat(v) + self.c_hat(v, w) + self.h_hat(w) < self.g_T(self.xgoal)) and \
                (self.g_T(v) + self.c_hat(v, w) < self.g_T(w))})
        return QV, QE

    def Prune(self, c, Xsamples, V, E):
        Xsamples = {x for x in Xsamples if self.f_hat(x) >= c}
        V.difference_update({v for v in V if self.f_hat(v) >=c})
        E.difference_update({(v, w) for (v, w) in E if (self.f_hat(v) > c) or (self.f_hat(w) > c)})
        Xsamples.update({v for v in V if self.g_T(v) == np.inf})
        V.difference_update({v for v in V if self.g_T(v) == np.inf})
        return Xsamples, V, E

    def radius(self, q):
        return 2 * self.eta * (1 + 1/self.n) ** (1/self.n) * \
            (self.Lambda(self.Xf_hat) / self.Zeta ) ** (1/self.n) * \
            (np.log(q) / q) ** (1/self.n)

    def Lambda(self, inputset):
        # lebesgue measure of a set, defined as 
        # mu: L(Rn) --> [0, inf], e.g. volume
        pass 

    def Zeta(self):
        # unit ball
        pass

    def BestInQueue(self, inputset):
        pass

    def BestQueueValue(self, inputset):
        pass

    def g_hat(self, v):
        pass

    def c(self, v, w):
        pass

    def c_hat(self, v, w):
        pass

    def f_hat(self, v):
        pass

    def h_hat(self, v):
        pass

    def g_T(self, v):
        pass
    