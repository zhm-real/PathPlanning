import numpy as np
import matplotlib.pyplot as plt

import os
import sys
from collections import defaultdict

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import StateSpace, getDist, getRay, isinbound, isinball
import pyrr


def getNearest(Space,pt):
    '''get the nearest point on the grid'''
    mindis,minpt = 1000,None
    for pts in Space: 
        dis = getDist(pts,pt)
        if dis < mindis:
            mindis,minpt = dis,pts
    return minpt

def isCollide(initparams, x, child):
    '''see if line intersects obstacle'''
    ray , dist = getRay(x, child) ,  getDist(x, child)
    if not isinbound(initparams.env.boundary,child):
        return True, dist
    for i in initparams.env.AABB:
        shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
        if shot is not None:
            dist_wall = getDist(x, shot)
            if dist_wall <= dist:  # collide
                return True, dist
    for i in initparams.env.balls:
        if isinball(i, child):
            return True, dist
        shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
        if shot != []:
            dists_ball = [getDist(x, j) for j in shot]
            if all(dists_ball <= dist):  # collide
                return True, dist
    return False, dist

def children(initparams, x):
    # get the neighbor of a specific state
    allchild = []
    resolution = initparams.env.resolution
    for direc in initparams.Alldirec:
        child = tuple(map(np.add,x,np.multiply(direc,resolution)))
        if isinbound(initparams.env.boundary,child):
            allchild.append(child)
    return allchild

def cost(initparams, x, y):
    # get the cost between two points, 
    # do collision check here
    collide, dist = isCollide(initparams,x,y)
    if collide: return np.inf
    else: return dist

def initcost(initparams):
    # initialize cost dictionary, could be modifed lateron
    c = defaultdict(lambda: defaultdict(dict)) # two key dicionary
    for xi in initparams.X:
        cdren = children(initparams, xi)
        for child in cdren:
            c[xi][child] = cost(initparams, xi, child)
    return c
    

class D_star(object):
    def __init__(self,resolution = 1):
        self.Alldirec = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
                                  [-1, 0, 0], [0, -1, 0], [0, 0, -1], [-1, -1, 0], [-1, 0, -1], [0, -1, -1],
                                  [-1, -1, -1],
                                  [1, -1, 0], [-1, 1, 0], [1, 0, -1], [-1, 0, 1], [0, 1, -1], [0, -1, 1],
                                  [1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]])
        self.env = env(resolution = resolution)
        self.X = StateSpace(self.env)
        self.x0, self.xt = getNearest(self.X, self.env.start), getNearest(self.X, self.env.goal)
        self.b = {} # back pointers every state has one except xt.
        self.OPEN = {} # OPEN list, here use a hashmap implementation. hash is point, key is value 
        self.h = self.initH() # estimate from a point to the end point
        self.tag = self.initTag() # set all states to new

        # initialize cost set
        self.c = initcost(self)

        # put G (ending state) into the OPEN list
        self.OPEN[self.xt] = 0

    def initH(self):
        # h set, all initialzed h vals are 0 for all states.
        h = {}
        for xi in self.X:
            h[xi] = 0
        return h

    def initTag(self):
        # tag , New point (never been in the OPEN list)
        #       Open point ( currently in OPEN )
        #       Closed (currently in CLOSED)
        t = {} 
        for xi in self.X:
            t[xi] = 'New'
        return t

    def get_kmin(self):
        # get the minimum of the k val in OPEN
        # -1 if it does not exist
        if self.OPEN:
            minv = np.inf
            for k,v in enumerate(self.OPEN):
                if v < minv: minv = v
            return minv
        return -1

    def min_state(self):
        # returns the state in OPEN with min k(.)
        # if empty, returns None and -1
        # it also removes this min value form the OPEN set.
        if self.OPEN:
            minv = np.inf
            for k,v in enumerate(self.OPEN):
                if v < minv: mink, minv = k, v
            return mink, self.OPEN.pop(mink)
        return None, -1
    
    def insert(self, x, h_new):
        # inserting a key and value into OPEN list (x, kx)
        # depending on following situations
        if self.tag[x] == 'New':
            kx = h_new
        if self.tag[x] == 'Open':
            kx = min(self.OPEN[x],h_new)
        if self.tag[x] == 'Closed':
            kx = min(self.h[x], h_new)
        self.OPEN[x] = kx
        self.h[x],self.tag[x] = h_new, 'Open'
            
    def process_state(self):
        x, kold = self.min_state()
        self.tag[x] = 'Closed'
        if x == None: return -1
        if kold < self.h[x]: # raised states
            for y in children(self,x):
                a = self.h[y] + self.c[y][x]
                if self.h[y] <= kold and self.h[x] > a:
                    self.b[x], self.h[x] = y , a
        elif kold == self.h[x]:# lower
            for y in children(self,x):
                bb = self.h[x] + self.c[x][y]
                if self.tag[y] == 'New' or \
                    (self.b[y] == x and self.h[y] != bb) or \
                    (self.b[y] != x and self.h[y] > bb):
                    self.b[y] = x
                    self.insert(y, bb)
        else: 
            for y in children(self,x):
                bb = self.h[x] + self.c[x][y]
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

    def modify_cost(self,x,y,cval):
        self.c[x][y] = cval # set the new cost to the cval 
        if self.tag[x] == 'Closed': self.insert(x,self.h[x])
        return self.get_kmin()

    def run(self):
        # TODO: implementation of changing obstable in process
        pass

        
if __name__ == '__main__':
    D = D_star(1)