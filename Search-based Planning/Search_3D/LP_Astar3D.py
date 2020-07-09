import numpy as np
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isinbound, isinball, hash3D, dehash, \
    cost, obstacleFree
from Search_3D.plot_util3D import visualization
import queue
import pyrr
import time


class Lifelong_Astar(object):
    def __init__(self,resolution = 1):
        self.Alldirec = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
                                  [-1, 0, 0], [0, -1, 0], [0, 0, -1], [-1, -1, 0], [-1, 0, -1], [0, -1, -1],
                                  [-1, -1, -1],
                                  [1, -1, 0], [-1, 1, 0], [1, 0, -1], [-1, 0, 1], [0, 1, -1], [0, -1, 1],
                                  [1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]])
        self.env = env(resolution=resolution)
        self.g = g_Space(self)
        self.start, self.goal = getNearest(self.g, self.env.start), getNearest(self.g, self.env.goal)
        self.x0, self.xt = hash3D(self.start), hash3D(self.goal)
        self.v = g_Space(self) # rhs(.) = g(.) = inf
        self.v[hash3D(self.start)] = 0 # rhs(x0) = 0
        self.h = Heuristic(self.g, self.goal)
        
        self.OPEN = queue.QueuePrior()  # store [point,priority]
        self.OPEN.put(self.x0, [self.h[self.x0],0])
        self.CLOSED = set()

        # used for A*
        self.done = False
        self.Path = []
        self.V = []
        self.ind = 0

        # initialize children list
        self.CHILDREN = {}
        self.getCHILDRENset()

        # initialize cost list
        self.COST = {}
        _ = self.costset()

    def costset(self):
        NodeToChange = set()
        for strxi in self.CHILDREN.keys():
            children = self.CHILDREN[strxi]
            xi = dehash(strxi)
            toUpdate = [self.cost(xj,xi) for xj in children]
            if strxi in self.COST:
                # if the old cost not equal to new cost
                diff = np.not_equal(self.COST[strxi],toUpdate)
                cd = np.array(children)[diff]
                for i in cd:
                    NodeToChange.add(hash3D(i))
                self.COST[strxi] = toUpdate
            else:
                self.COST[strxi] = toUpdate
        return NodeToChange

    def getCOSTset(self,strxi,xj):
        ind, children = 0, self.CHILDREN[strxi]
        for i in children:
            if all(i == xj):
                return self.COST[strxi][ind]
            ind += 1
            

    def children(self, x):
        allchild = []
        resolution = self.env.resolution
        for direc in self.Alldirec:
            child = np.array(list(map(np.add,x,np.multiply(direc,resolution))))
            if isinbound(self.env.boundary,child):
                allchild.append(child)
        return allchild

    def getCHILDRENset(self):
        for strxi in self.g.keys():
            xi = dehash(strxi)
            self.CHILDREN[strxi] = self.children(xi)
        
    def isCollide(self, x, child):
        ray , dist = getRay(x, child) ,  getDist(x, child)
        if not isinbound(self.env.boundary,child):
            return True, dist
        for i in self.env.AABB:
            shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
            if shot is not None:
                dist_wall = getDist(x, shot)
                if dist_wall <= dist:  # collide
                    return True, dist
        for i in self.env.balls:
            if isinball(i, child):
                return True, dist
            shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
            if shot != []:
                dists_ball = [getDist(x, j) for j in shot]
                if all(dists_ball <= dist):  # collide
                    return True, dist
        return False, dist

    def cost(self, x, y):
        collide, dist = self.isCollide(x, y)
        if collide: return np.inf
        else: return dist
            
    def key(self,strxi,epsilion = 1):
        return [min(self.g[strxi],self.v[strxi]) + epsilion*self.h[strxi],min(self.g[strxi],self.v[strxi])]

    def path(self):
        path = []
        strx = self.xt
        strstart = self.x0
        ind = 0
        while strx != strstart:
            j = dehash(strx)
            nei = self.CHILDREN[strx]
            gset = [self.g[hash3D(xi)] for xi in nei]
            # collision check and make g cost inf
            for i in range(len(nei)):
                if self.isCollide(nei[i],j)[0]:
                    gset[i] = np.inf
            parent = nei[np.argmin(gset)]
            path.append([dehash(strx), parent])
            strx = hash3D(parent)
            if ind > 100:
                break
            ind += 1
        return path

    #------------------Lifelong Plannning A* 
    def UpdateMembership(self,strxi, xi, xparent=None):
        if strxi != self.x0:
            self.v[strxi] = min([self.g[hash3D(j)] + self.getCOSTset(strxi,j) for j in self.CHILDREN[strxi]])
        self.OPEN.check_remove(strxi)
        if self.g[strxi] != self.v[strxi]:
            self.OPEN.put(strxi,self.key(strxi))
    
    def ComputePath(self):
        print('computing path ...')
        while self.key(self.xt) > self.OPEN.top_key() or self.v[self.xt] != self.g[self.xt]:
            strxi = self.OPEN.get()
            xi = dehash(strxi)
            # if g > rhs, overconsistent
            if self.g[strxi] > self.v[strxi]: 
                self.g[strxi] = self.v[strxi]
                # add xi to expanded node set
                if strxi not in self.CLOSED:
                    self.V.append(xi)
                self.CLOSED.add(strxi)
            else: # underconsistent and consistent
                self.g[strxi] = np.inf
                self.UpdateMembership(strxi, xi)
            for xj in self.CHILDREN[strxi]:
                strxj = hash3D(xj)
                self.UpdateMembership(strxj, xj)

            # visualization(self)
            self.ind += 1
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(2)

    def change_env(self):
        self.env.change()
        self.done = False
        self.Path = []
        self.CLOSED = set()
        N = self.costset()
        for strxi in N:
            xi = dehash(strxi)
            self.UpdateMembership(strxi,xi)

if __name__ == '__main__':
    sta = time.time()
    Astar = Lifelong_Astar(1)
    Astar.ComputePath()
    Astar.change_env()
    Astar.ComputePath()
    plt.show()
    print(time.time() - sta)