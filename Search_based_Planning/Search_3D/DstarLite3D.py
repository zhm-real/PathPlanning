import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")
from Search_3D.env3D import env
from Search_3D.utils3D import getDist, heuristic_fun, getNearest, isinbound, \
     cost, children, StateSpace
from Search_3D.plot_util3D import visualization
from Search_3D import queue
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
        #self.X = StateSpace(self.env)
        #self.x0, self.xt = getNearest(self.X, self.env.start), getNearest(self.X, self.env.goal)
        self.settings = 'CollisionChecking' # for collision checking
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        # self.OPEN = queue.QueuePrior()
        self.OPEN = queue.MinheapPQ()
        self.km = 0
        self.g = {} # all g initialized at inf
        self.rhs = {self.xt:0} # rhs(x0) = 0
        self.h = {}
        self.OPEN.put(self.xt, self.CalculateKey(self.xt))
        self.CLOSED = set()
        
        # init children set:
        self.CHILDREN = {}
        # init Cost set
        self.COST = defaultdict(lambda: defaultdict(dict))
        
        # for visualization
        self.V = set()  # vertice in closed
        self.ind = 0
        self.Path = []
        self.done = False

    def updatecost(self, range_changed=None, new=None, old=None, mode=False):
        # scan graph for changed Cost, if Cost is changed update it
        CHANGED = set()
        for xi in self.CLOSED:
            if isinbound(old, xi, mode) or isinbound(new, xi, mode):
                newchildren = set(children(self, xi))  # B
                self.CHILDREN[xi] = newchildren
                for xj in newchildren:
                    self.COST[xi][xj] = cost(self, xi, xj)
                CHANGED.add(xi)
        return CHANGED

    def getcost(self, xi, xj):
        # use a LUT for getting the costd
        if xi not in self.COST:
            for (xj,xjcost) in children(self, xi, settings=1):
                self.COST[xi][xj] = cost(self, xi, xj, xjcost)
        # this might happen when there is a node changed. 
        if xj not in self.COST[xi]:
            self.COST[xi][xj] = cost(self, xi, xj)
        return self.COST[xi][xj]

    def getchildren(self, xi):
        if xi not in self.CHILDREN:
            allchild = children(self, xi)
            self.CHILDREN[xi] = set(allchild)
        return self.CHILDREN[xi]

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
        if not getDist(self.xt, u) <= self.env.resolution: # originally: u != x_goal
            if u in self.CHILDREN and len(self.CHILDREN[u]) == 0:
                self.rhs[u] = np.inf
            else:
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
            if not self.done: # first time running, we need to stop on this condition
                if getDist(self.x0,u) < 1*self.env.resolution:
                    self.x0 = u
                    break
            if kold < self.CalculateKey(u):
                self.OPEN.put(u, self.CalculateKey(u))
            if self.getg(u) > self.getrhs(u):
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = np.inf
                self.UpdateVertex(u)
            for s in self.getchildren(u):
                self.UpdateVertex(s)
            # visualization(self)
            self.ind += 1

    def main(self):
        s_last = self.x0
        print('first run ...')
        self.ComputeShortestPath()
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(0.5)
        # plt.show()
        print('running with map update ...')
        t = 0 # count time
        ischanged = False
        self.V = set()
        while getDist(self.x0, self.xt) > 2*self.env.resolution:
            #---------------------------------- at specific times, the environment is changed and Cost is updated
            if t % 2 == 0: 
                new0,old0 = self.env.move_block(a=[-0.1, 0, -0.2], s=0.5, block_to_move=1, mode='translation')
                new1,old1 = self.env.move_block(a=[0, 0, -0.2], s=0.5, block_to_move=0, mode='translation')
                new2,old2 = self.env.move_OBB(theta = [0,0.1*t,0])
                #new2,old2 = self.env.move_block(a=[-0.3, 0, -0.1], s=0.5, block_to_move=1, mode='translation')
                ischanged = True
                self.Path = []
            #----------------------------------- traverse the route as originally planned
            if t == 0:
                children_new = [i for i in self.CLOSED if getDist(self.x0, i) <= self.env.resolution*np.sqrt(3)]
            else:
                children_new = list(children(self,self.x0))
            self.x0 = children_new[np.argmin([self.getcost(self.x0,s_p) + self.getg(s_p) for s_p in children_new])]
            # TODO add the moving robot position codes
            self.env.start = self.x0
            # ---------------------------------- if any Cost changed, update km, reset slast,
            #                                    for all directed edgees (u,v) with  chaged edge costs, 
            #                                    update the edge Cost cBest(u,v) and update vertex u. then replan
            if ischanged:
                self.km += heuristic_fun(self, self.x0, s_last)
                s_last = self.x0
                CHANGED = self.updatecost(True, new0, old0)
                CHANGED1 = self.updatecost(True, new1, old1)
                CHANGED2 = self.updatecost(True, new2, old2, mode='obb')
                CHANGED = CHANGED.union(CHANGED1, CHANGED2)
                # self.V = set()
                for u in CHANGED:
                    self.UpdateVertex(u)
                self.ComputeShortestPath()
                
                ischanged = False
            self.Path = self.path(self.x0)
            visualization(self)
            t += 1
        plt.show()

    def path(self, s_start=None):
        '''After ComputeShortestPath()
        returns, one can then follow a shortest path from x_init to
        x_goal by always moving from the current vertex s, starting
        at x_init. , to any successor s' that minimizes cBest(s,s') + g(s')
        until x_goal is reached (ties can be broken arbitrarily).'''
        path = []
        s_goal = self.xt
        if not s_start:
            s = self.x0
        else:
            s= s_start
        ind = 0
        while s != s_goal:
            if s == self.x0:
                children = [i for i in self.CLOSED if getDist(s, i) <= self.env.resolution*np.sqrt(3)]
            else: 
                children = list(self.CHILDREN[s])
            snext = children[np.argmin([self.getcost(s,s_p) + self.getg(s_p) for s_p in children])]
            path.append([s, snext])
            s = snext
            if ind > 100:
                break
            ind += 1
        return path

if __name__ == '__main__':
    
    D_lite = D_star_Lite(1)
    a = time.time()
    D_lite.main()
    print('used time (s) is ' + str(time.time() - a))
            