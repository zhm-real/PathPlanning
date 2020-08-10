# This is Advanced Batched Informed Tree star 3D algorithm 
# implementation
"""
This is ABIT* code for 3D
@author: yue qi 
source: M.P.Strub, J.D.Gammel. "Advanced BIT* (ABIT*):
        Sampling-Based Planning with Advanced Graph-Search Techniques" 
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

class ABIT_star:

    def __init__(self):
        self.env = env()
        self.xstart, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.maxiter = 1000
        self.done = False
        self.n = 1000# used in radius calc r(q)
        self.lam = 10 # used in radius calc r(q)

    def run(self):
        V, E = {self.xstart}, set()
        T = (V,E)
        Xunconnected = {self.xgoal}
        q = len(V) + len(Xunconnected)
        eps_infl, eps_trunc = np.inf, np.inf
        Vclosed, Vinconsistent = set(), set()
        Q = self.expand(self.xstart, T, Xunconnected, np.inf)

        ind = 0
        while True:
            if self.is_search_marked_finished():
                if self.update_approximation(eps_infl, eps_trunc):
                    T, Xunconnected = self.prune(T, Xunconnected, self.xgoal)
                    Xunconnected.update(self.sample(m, self.xgoal))
                    q = len(V) + len(Xunconnected)
                    Q = self.expand({self.xstart}, T, Xunconnected, self.r(q))
                else:
                    Q.update(self.expand(Vinconsistent, T, Xunconnected, self.r(q)))
                eps_infl = self.update_inflation_factor()
                eps_trunc = self.update_truncation_factor()
                Vclosed = set()
                Vinconsistent = set()
                self.mark_search_unfinished()
            else:
                state_tuple = list(Q)
                (xp, xc) = state_tuple[np.argmin( [self.g_T[xi] + self.c_hat(xi,xj) + eps_infl * self.h_hat(xj) for (xi,xj) in Q] )]
                Q = Q.difference({(xp, xc)})
                if (xp, xc) in E:
                    if xc in Vclosed:
                        Vinconsistent.add(xc)
                    else:
                        Q.update(self.expand({xc}, T, Xunconnected, self.r(q)))
                        Vclosed.add(xc)
                elif eps_trunc * (self.g_T(xp) + self.c_hat(xi, xj) + self.h_hat(xc)) <= self.g_T(self.xgoal):
                    if self.g_T(xp) + self.c_hat(xp, xc) < self.g_T(xc):
                        if self.g_T(xp) + self.c(xp, xc) + self.h_hat(xc) < self.g_T(self.xgoal):
                            if self.g_T(xp) + self.c(xp, xc) < self.g_T(xc):
                                if xc in V:
                                    E = E.difference({(xprev, xc)})
                                else:
                                    Xunconnected.difference_update({xc})
                                    V.add(xc)
                                    E.add((xp, xc))
                                if xc in Vclosed:
                                    Vinconsistent.add(xc)
                                else:
                                    Q.update(self.expand({xc}, T, Xunconnected, self.r(q)))
                                    Vclosed.add(xc)
                else: 
                    self.mark_search_finished()
            ind += 1
            # until stop
            if ind > self.maxiter:
                break

    def sample(self, m, xgoal):
        pass

    def expand(self, set_xi, T, Xunconnected, r):
        V, E = T
        Eout = set()
        for xp in set_xi:
            Eout.update({(x1, x2) for (x1, x2) in E if x1 == xp})
            for xc in {x for x in Xunconnected.union(V) if getDist(xp, x) <= r}:
                if self.g_hat(xp) + self.c_hat(xp, xc) + self.h_hat(xc) <= self.g_T(self.xgoal):
                    if self.g_hat(xp) + self.c_hat(xp, xc) <= self.g_hat(xc):
                        Eout.add((xp,xc))
        return Eout

    def prune(self, T, Xunconnected, xgoal):
        V, E = T
        Xunconnected.difference_update({x for x in Xunconnected if self.f_hat(x) >= self.g_T(xgoal)})
        V.difference_update({x for x in V if self.f_hat(x) > self.g_T(xgoal)})
        E.difference_update({(xp, xc) for (xp, xc) in E if self.f_hat(xp) > self.g_T(xgoal) or self.f_hat(xc) > self.g_T(xgoal)})
        Xunconnected.update({xc for (xp, xc) in E if (xp not in V) and (xc in V)})
        V.difference_update({xc for (xp, xc) in E if (xp not in V) and (xc in V)})
        T = (V,E)
        return T, Xunconnected

    def g_hat(self, x):
        pass
    
    def h_hat(self, x):
        pass

    def c_hat(self, x1, x2):
        pass

    def f_hat(self, x):
        pass

    def g_T(self, x):
        pass

    def r(self, q):
        return self.eta * (2 * (1 + 1/self.n) * (self.Lambda(self.Xf_hat) / self.Zeta) * (np.log(q) / q)) ** (1 / self.n)

    def Lambda(self, inputset):
        pass

    def Zeta(self):
        pass

    def is_search_marked_finished(self):
        return self.done

    def mark_search_unfinished(self):
        self.done = False
        return self.done

    def mark_search_finished(self):
        self.done = True
        return self.done
