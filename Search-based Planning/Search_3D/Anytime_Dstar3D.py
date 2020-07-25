# check paper of 
# [Likhachev2005]
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D.utils3D import getDist, heuristic_fun, getNearest, isinbound, \
     cost, children, StateSpace
from Search_3D.plot_util3D import visualization
from Search_3D import queue
import time

class Anytime_Dstar(object):

    def __init__(self, resolution=1):
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
        self.settings = 'CollisionChecking' # for collision checking
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.OPEN = queue.MinheapPQ()
        self.g = {} # all g initialized at inf
        self.h = {}
        self.rhs = {self.xt:0} # rhs(x0) = 0
        self.OPEN.put(self.xt, self.key(self.xt))
        self.INCONS = set()
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

        # epsilon in the key caculation
        self.epsilon = 1
        self.increment = 0.1
        self.decrement = 0.2

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

    def updatecost(self,range_changed=None, new=None, old=None, mode=False):
        # scan graph for changed cost, if cost is changed update it
        CHANGED = set()
        for xi in self.CLOSED:
            if xi in self.CHILDREN:
                oldchildren = self.CHILDREN[xi]# A
                if isinbound(old, xi, mode) or isinbound(new, xi, mode):
                    newchildren = set(children(self,xi))# B
                    removed = oldchildren.difference(newchildren)
                    intersection = oldchildren.intersection(newchildren)
                    added = newchildren.difference(oldchildren)
                    for xj in removed:
                        self.COST[xi][xj] = cost(self, xi, xj)
                    for xj in intersection.union(added):
                        self.COST[xi][xj] = cost(self, xi, xj)
                    CHANGED.add(xi)
            else: 
                if isinbound(old, xi, mode) or isinbound(new, xi, mode):
                    CHANGED.add(xi)
                    children_added = set(children(self,xi))
                    self.CHILDREN[xi] = children_added
                    for xj in children_added:
                        self.COST[xi][xj] = cost(self, xi, xj)
        return CHANGED
#--------------main functions for Anytime D star

    def key(self, s, epsilon=1):
        if self.getg(s) > self.getrhs(s):
            return [self.rhs[s] + epsilon * heuristic_fun(self, s, self.x0), self.rhs[s]]
        else:
            return [self.getg(s) + heuristic_fun(self, s, self.x0), self.getg(s)]

    def UpdateState(self, s):
        if s not in self.CLOSED:
        # TODO if s is not visited before
            self.g[s] = np.inf
        if s != self.xt:
            self.rhs[s] = min([self.getcost(s, s_p) + self.getg(s_p) for s_p in self.getchildren(s)]) 
        self.OPEN.check_remove(s)
        if self.getg(s) != self.getrhs(s):
            if s not in self.CLOSED:
                self.OPEN.put(s, self.key(s))
            else:
                self.INCONS.add(s)

    def ComputeorImprovePath(self):
        while self.OPEN.top_key() < self.key(self.x0,self.epsilon) or self.rhs[self.x0] != self.g[self.x0]:
            s = self.OPEN.get()

            if getDist(s, tuple(self.env.start)) < self.env.resolution:
                break

            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                self.CLOSED.add(s)
                self.V.add(s)
                for s_p in self.getchildren(s):
                    self.UpdateState(s_p)
            else:
                self.g[s] = np.inf
                self.UpdateState(s)
                for s_p in self.getchildren(s):
                    self.UpdateState(s_p)
            self.ind += 1

    def Main(self):
        ischanged = False
        islargelychanged = False
        t = 0
        self.ComputeorImprovePath()
        #TODO publish current epsilon sub-optimal solution
        self.done = True
        self.ind = 0
        self.Path = self.path()
        visualization(self)
        while True:
            visualization(self)
            if t == 20:
                break
            # change environment
            # new2,old2 = self.env.move_block(theta = [0,0,0.1*t], mode='rotation')
            new2,old2 = self.env.move_block(a = [0,0,-0.2], mode='translation')
            ischanged = True
            # islargelychanged = True
            self.Path = []

            # update cost with changed environment
            if ischanged:
                # CHANGED = self.updatecost(True, new2, old2, mode='obb')
                CHANGED = self.updatecost(True, new2, old2)
                for u in CHANGED:
                    self.UpdateState(u)
                self.ComputeorImprovePath()
                ischanged = False

            if islargelychanged:  
                self.epsilon += self.increment # or replan from scratch
            elif self.epsilon > 1:
                self.epsilon -= self.decrement
            
            # move states from the INCONS to OPEN
            # update priorities in OPEN
            Allnodes = self.INCONS.union(self.OPEN.allnodes())
            for node in Allnodes:
                self.OPEN.put(node, self.key(node, self.epsilon)) 
            self.INCONS = set()
            self.CLOSED = set()
            self.ComputeorImprovePath()
            #TODO publish current epsilon sub optimal solution
            self.Path = self.path()
            # if epsilon == 1:
                # wait for change to occur
            t += 1

    def path(self, s_start=None):
        '''After ComputeShortestPath()
        returns, one can then follow a shortest path from s_start to
        s_goal by always moving from the current vertex s, starting
        at s_start. , to any successor s' that minimizes c(s,s') + g(s') 
        until s_goal is reached (ties can be broken arbitrarily).'''
        path = []
        s_goal = self.xt
        s = self.x0
        ind = 0
        while getDist(s, s_goal) > self.env.resolution:
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
    AD = Anytime_Dstar(resolution = 1)
    AD.Main()