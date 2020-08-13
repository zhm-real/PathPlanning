import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import StateSpace, getDist, getNearest, getRay, isinbound, isinball, isCollide, children, cost, \
    initcost
from Search_3D.plot_util3D import visualization


class D_star(object):
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
        self.settings = 'CollisionChecking'
        self.env = env(resolution=resolution)
        self.X = StateSpace(self.env)
        self.x0, self.xt = getNearest(self.X, self.env.start), getNearest(self.X, self.env.goal)
        # self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.b = defaultdict(lambda: defaultdict(dict))  # back pointers every state has one except xt.
        self.OPEN = {}  # OPEN list, here use a hashmap implementation. hash is point, key is value
        self.h = {}  # estimate from a point to the end point
        self.tag = {}  # set all states to new
        self.V = set()  # vertice in closed
        # for visualization
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}

    def checkState(self, y):
        if y not in self.h:
            self.h[y] = 0
        if y not in self.tag:
            self.tag[y] = 'New'

    def get_kmin(self):
        # get the minimum of the k val in OPEN
        # -1 if it does not exist
        if self.OPEN:
            return min(self.OPEN.values())
        return -1

    def min_state(self):
        # returns the state in OPEN with min k(.)
        # if empty, returns None and -1
        # it also removes this min value form the OPEN set.
        if self.OPEN:
            minvalue = min(self.OPEN.values())
            for k in self.OPEN.keys():
                if self.OPEN[k] == minvalue:
                    return k, self.OPEN.pop(k)
        return None, -1

    def insert(self, x, h_new):
        # inserting a key and value into OPEN list (s, kx)
        # depending on following situations
        if self.tag[x] == 'New':
            kx = h_new
        if self.tag[x] == 'Open':
            kx = min(self.OPEN[x], h_new)
        if self.tag[x] == 'Closed':
            kx = min(self.h[x], h_new)
        self.OPEN[x] = kx
        self.h[x], self.tag[x] = h_new, 'Open'

    def process_state(self):
        # main function of the D star algorithm, perform the process state 
        # around the old path when needed.
        x, kold = self.min_state()
        self.tag[x] = 'Closed'
        self.V.add(x)
        if x is None:
            return -1
        # check if 1st timer s
        self.checkState(x)
        if kold < self.h[x]:  # raised states
            for y in children(self, x):
                # check y
                self.checkState(y)
                a = self.h[y] + cost(self, y, x)
                if self.h[y] <= kold and self.h[x] > a:
                    self.b[x], self.h[x] = y, a
        if kold == self.h[x]:  # lower
            for y in children(self, x):
                # check y
                self.checkState(y)
                bb = self.h[x] + cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb) or \
                        (self.b[y] != x and self.h[y] > bb):
                    self.b[y] = x
                    self.insert(y, bb)
        else:
            for y in children(self, x):
                 # check y
                self.checkState(y)
                bb = self.h[x] + cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb):
                    self.b[y] = x
                    self.insert(y, bb)
                else:
                    if self.b[y] != x and self.h[y] > bb:
                        self.insert(x, self.h[x])
                    else:
                        if self.b[y] != x and self.h[y] > bb and \
                                self.tag[y] == 'Closed' and self.h[y] == kold:
                            self.insert(y, self.h[y])
        return self.get_kmin()

    def modify_cost(self, x):
        xparent = self.b[x]
        if self.tag[x] == 'Closed':
            self.insert(x, self.h[xparent] + cost(self, x, xparent))

    def modify(self, x):
        self.modify_cost(x)
        while True:
            kmin = self.process_state()
            # visualization(self)
            if kmin >= self.h[x]:
                break

    def path(self, goal=None):
        path = []
        if not goal:
            x = self.x0
        else:
            x = goal
        start = self.xt
        while x != start:
            path.append([np.array(x), np.array(self.b[x])])
            x = self.b[x]
        return path

    def run(self):
        # put G (ending state) into the OPEN list
        self.OPEN[self.xt] = 0
        self.tag[self.x0] = 'New'
        # first run
        while True:
            # TODO: self.x0 =
            self.process_state()
            # visualization(self)
            if self.tag[self.x0] == "Closed":
                break
            self.ind += 1
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(0.2)
        # plt.show()
        # when the environemnt changes over time

        for i in range(5):
            self.env.move_block(a=[0, -0.50, 0], s=0.5, block_to_move=1, mode='translation')
            self.env.move_block(a=[-0.25, 0, 0], s=0.5, block_to_move=0, mode='translation')
            # travel from end to start
            s = tuple(self.env.start)
            # self.V = set()
            while s != self.xt:
                if s == tuple(self.env.start):
                    sparent = self.b[self.x0]
                else:
                    sparent = self.b[s]
                # if there is a change of Cost, or a collision.
                if cost(self, s, sparent) == np.inf:
                    self.modify(s)
                    continue
                self.ind += 1
                s = sparent
            self.Path = self.path()
            visualization(self)
        plt.show()


if __name__ == '__main__':
    D = D_star(1)
    D.run()
