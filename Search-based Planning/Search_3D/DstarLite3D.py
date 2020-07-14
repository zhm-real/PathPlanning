import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, heuristic_fun, getNearest, isinbound, isinball, \
    cost, obstacleFree, children, StateSpace
from Search_3D.plot_util3D import visualization
import queue
import pyrr
import time

class D_star_Lite(object):
    # Original version of the D*lite
    def __init__(self, resolution = 1):
        self.Alldirec = {(1, 0, 0): 1, (0, 1, 0): 1, (0, 0, 1): 1, \
                        (-1, 0, 0): 1, (0, -1, 0): 1, (0, 0, -1): 1, \
                        (1, 1, 0): np.sqrt(2), (1, 0, 1): np.sqrt(2), (0, 1, 1): np.sqrt(2), \
                        (-1, -1, 0): np.sqrt(2), (-1, 0, -1): np.sqrt(2), (0, -1, -1): np.sqrt(2), \
                        (1, -1, 0): np.sqrt(2), (-1, 1, 0): np.sqrt(2), (1, 0, -1): np.sqrt(2), \
                        (-1, 0, 1): np.sqrt(2), (0, 1, -1): np.sqrt(2), (0, -1, 1): np.sqrt(2), \
                        (1, 1, 1): np.sqrt(3), (-1, -1, -1) : np.sqrt(3), \
                        (1, -1, -1): np.sqrt(3), (-1, 1, -1): np.sqrt(3), (-1, -1, 1): np.sqrt(3), \
                        (1, 1, -1): np.sqrt(3), (1, -1, 1): np.sqrt(3), (-1, 1, 1): np.sqrt(3)}
        self.env = env(resolution=resolution)
        # self.X = StateSpace(self.env)
        # self.x0, self.xt = getNearest(self.X, self.env.start), getNearest(self.X, self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.OPEN = queue.QueuePrior()
        self.km = 0
        self.g = {} # all g initialized at inf
        self.rhs = {self.xt:0} # rhs(x0) = 0
        self.h = {}
        self.OPEN.put(self.xt, self.CalculateKey(self.xt))

        # init children set:
        self.CHILDREN = {}
        # init cost set
        self.COST = defaultdict(lambda: defaultdict(dict))

        # for visualization
        self.V = set()  # vertice in closed
        self.ind = 0
        self.Path = []
        self.done = False

    def getcost(self, xi, xj):
        # use a LUT for getting the costd
        if xi not in self.COST:
            for (xj,xjcost) in children(self, xi, settings=1):
                self.COST[xi][xj] = cost(self, xi, xj, xjcost)
        # this might happen when there is a node changed. 
        if xj not in self.COST[xi]:
            self.COST[xi][xj] = cost(self, xi, xj)
        return self.COST[xi][xj]

    def updatecost(self):
        # TODO: update cost when the environment is changed
        pass

    def getchildren(self, xi):
        if xi not in self.CHILDREN:
            allchild = children(self, xi)
            self.CHILDREN[xi] = set(allchild)
        return self.CHILDREN[xi]

    def updatechildren(self):
        # TODO: update children set when the environment is changed
        pass

    def geth(self, xi):
        # when the heurisitic is first calculated
        if xi not in self.h:
            self.h[xi] = heuristic_fun(self, xi, self.x0)
        return self.h[xi]

    def getg(self, xi):
        if xi not in self.g:
            self.g[xi] = np.inf
        return self.g[xi]

    def getrhs(self, xi):
        if xi not in self.rhs:
            self.rhs[xi] = np.inf
        return self.rhs[xi]
#-------------main functions for D*Lite-------------

    def CalculateKey(self, s, epsilion = 1):
        return [min(self.getg(s), self.getrhs(s)) + epsilion * self.geth(s) + self.km, min(self.getg(s), self.getrhs(s))]

    def UpdateVertex(self, u):
        # if still in the hunt
        if not getDist(self.xt, u) <= self.env.resolution: # originally: u != s_goal
            self.rhs[u] = min([self.getcost(s, u) + self.getg(s) for s in self.getchildren(u)])
        # if u is in OPEN, remove it
        self.OPEN.check_remove(u)
        # if rhs(u) not equal to g(u)
        if self.getg(u) != self.getrhs(u):
            self.OPEN.put(u, self.CalculateKey(u))
        
    def ComputeShortestPath(self):
        while self.OPEN.top_key() < self.CalculateKey(self.x0) or self.getrhs(self.x0) != self.getg(self.x0) :
            kold = self.OPEN.top_key()
            u = self.OPEN.get()
            self.V.add(u)
            if getDist(self.x0, u) <= self.env.resolution:
                break
            # visualization(self)
            if kold < self.CalculateKey(u):
                self.OPEN.put(u, self.CalculateKey(u))
            if self.getg(u) > self.getrhs(u):
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = np.inf
                self.UpdateVertex(u)
            for s in self.getchildren(u):
                self.UpdateVertex(s)

            self.ind += 1

    def main(self):
        s_last = self.x0
        s_start = self.x0
        self.ComputeShortestPath()
        # while s_start != self.xt:
        # while getDist(s_start, self.xt) > self.env.resolution:
        #     newcost, allchild = [], []
        #     for i in children(self, s_start):
        #         newcost.append(cost(self, i, s_start) + self.g[s_start])
        #         allchild.append(i)
        #     s_start = allchild[np.argmin(newcost)]
        #     #TODO: move to s_start
        #     #TODO: scan graph or costs changes 
        #     # self.km = self.km + heuristic_fun(self, s_start, s_last)
        #     # for all directed edges (u,v) with changed edge costs
        #     #   update edge cost c(u,v)
        #     #   updatevertex(u)
        #     self.ComputeShortestPath()

if __name__ == '__main__':
    a = time.time()
    D_lite = D_star_Lite(1)
    # D_lite.UpdateVertex(D_lite.x0)
    D_lite.main()
    print('used time (s) is ' + str(time.time() - a))
            