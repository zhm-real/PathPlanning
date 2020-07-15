import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, heuristic_fun, getNearest, isinbound, isinball, \
    isCollide, cost, obstacleFree, children, StateSpace
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
        self.CLOSED = set()
        
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

    def updatecost(self,range_changed=None):
        # TODO: update cost when the environment is changed
        # chaged nodes
        CHANGED = set()
        for xi in self.CLOSED:
            oldchildren = self.CHILDREN[xi]# A
            # if you don't know where the change occured:
            if range_changed is None:
                newchildren = set(children(self,xi))# B
                added = newchildren.difference(oldchildren)# B-A
                removed = oldchildren.difference(newchildren)# A-B
                self.CHILDREN[xi] = newchildren
                if added or removed:
                    CHANGED.add(xi)
                for xj in removed:
                    self.COST[xi][xj] = cost(self, xi, xj)  
                for xj in added:
                    self.COST[xi][xj] = cost(self, xi, xj)    
            # if you do know where on the map changed, only update those changed around that area
            else: 
                if isinbound(range_changed, xi):
                    newchildren = set(children(self,xi))# B
                    added = newchildren.difference(oldchildren)# B-A
                    removed = oldchildren.difference(newchildren)# A-B
                    self.CHILDREN[xi] = newchildren
                    if added or removed:
                        CHANGED.add(xi)
                    for xj in removed:
                        self.COST[xi][xj] = cost(self, xi, xj)  
                    for xj in added:
                        self.COST[xi][xj] = cost(self, xi, xj)         
        return CHANGED

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
            self.CLOSED.add(u)
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
        print('first run ...')
        self.ComputeShortestPath()
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(0.5)
        # plt.show()
        # change the environment 
        print('running with map update ...')
        for i in range(100):
            range_changed1 = self.env.move_block(a=[0, 0, -0.1], s=0.5, block_to_move=0, mode='translation')
            range_changed2 = self.env.move_block(a=[0.1, 0, 0], s=0.5, block_to_move=1, mode='translation')
            range_changed3 = self.env.move_block(a=[0, 0.1, 0], s=0.5, block_to_move=2, mode='translation')
            #range_changed = self.env.move_block(a=[0.1, 0, 0], s=0.5, block_to_move=1, mode='translation')
                #   update the edge cost of c(u,v) 
            CHANGED1 = self.updatecost(range_changed1)
            CHANGED2 = self.updatecost(range_changed2)
            CHANGED3 = self.updatecost(range_changed3)
            CHANGED2 = CHANGED2.union(CHANGED1)
            CHANGED = CHANGED3.union(CHANGED2)
            while getDist(s_start, self.xt) > 2*self.env.resolution:
                if s_start == self.x0:
                    children = [i for i in self.CLOSED if getDist(s_start, i) <= self.env.resolution*np.sqrt(3)]
                else:
                    children = list(self.CHILDREN[s_start])
                s_start = children[np.argmin([cost(self,s_start,s_p) + self.g[s_p] for s_p in children])]
                
                #   for all directed edges (u,v) with changed costs
                if CHANGED:
                    self.km = self.km + heuristic_fun(self, s_start, s_last)
                    for u in CHANGED:
                        self.UpdateVertex(u)
                    s_last = s_start
                    self.ComputeShortestPath()
            self.Path = self.path()
            visualization(self)
        plt.show()

    def path(self):
        '''After ComputeShortestPath()
        returns, one can then follow a shortest path from s_start to
        s_goal by always moving from the current vertex s, starting
        at s_start. , to any successor s' that minimizes c(s,s') + g(s') 
        until s_goal is reached (ties can be broken arbitrarily).'''
        path = []
        s_goal = self.xt
        s = self.x0
        ind = 0
        while s != s_goal:
            if s == self.x0:
                children = [i for i in self.CLOSED if getDist(s, i) <= self.env.resolution*np.sqrt(3)]
            else: 
                children = list(self.CHILDREN[s])
            snext = children[np.argmin([cost(self,s,s_p) + self.g[s_p] for s_p in children])]
            path.append([s, snext])
            s = snext
            if ind > 100:
                break
            ind += 1
        return path

if __name__ == '__main__':
    
    D_lite = D_star_Lite(1)
    #D_lite.ComputeShortestPath()
    a = time.time()
    #range_changed = D_lite.env.move_block(a=[0, 0, 1], s=0.5, block_to_move=1, mode='translation')
    #CHANGED = D_lite.updatecost(range_changed)
    D_lite.main()
    print('used time (s) is ' + str(time.time() - a))
            