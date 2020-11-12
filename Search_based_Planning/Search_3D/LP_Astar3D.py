import numpy as np
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay, g_Space, Heuristic, getNearest, isinbound, isinball, \
    cost, obstacleFree, isCollide
from Search_3D.plot_util3D import visualization
import queue
import pyrr
import time


class Lifelong_Astar(object):
    def __init__(self,resolution = 1):
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
        self.g = g_Space(self)
        self.start, self.goal = getNearest(self.g, self.env.start), getNearest(self.g, self.env.goal)
        self.x0, self.xt = self.start, self.goal
        self.rhs = g_Space(self) # rhs(.) = g(.) = inf
        self.rhs[self.start] = 0 # rhs(x0) = 0
        self.h = Heuristic(self.g, self.goal)
        
        self.OPEN = queue.MinheapPQ()  # store [point,priority]
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

        # initialize Cost list
        self.COST = {}
        _ = self.costset()

    def costset(self):
        NodeToChange = set()
        for xi in self.CHILDREN.keys():
            children = self.CHILDREN[xi]
            toUpdate = [self.cost(xj,xi) for xj in children]
            if xi in self.COST:
                # if the old Cost not equal to new Cost
                diff = np.not_equal(self.COST[xi],toUpdate)
                cd = np.array(children)[diff]
                for i in cd:
                    NodeToChange.add(tuple(i))
                self.COST[xi] = toUpdate
            else:
                self.COST[xi] = toUpdate
        return NodeToChange

    def getCOSTset(self,xi,xj):
        ind, children = 0, self.CHILDREN[xi]
        for i in children:
            if i == xj:
                return self.COST[xi][ind]
            ind += 1
            

    def children(self, x):
        allchild = []
        resolution = self.env.resolution
        for direc in self.Alldirec:
            child = tuple(map(np.add,x,np.multiply(direc,resolution)))
            if isinbound(self.env.boundary,child):
                allchild.append(child)
        return allchild

    def getCHILDRENset(self):
        for xi in self.g.keys():
            self.CHILDREN[xi] = self.children(xi)
        
    def isCollide(self, x, child):
        ray , dist = getRay(x, child) ,  getDist(x, child)
        if not isinbound(self.env.boundary,child):
            return True, dist
        for i in self.env.AABB_pyrr:
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
        collide, dist = isCollide(self, x, y)
        if collide: return np.inf
        else: return dist
            
    def key(self,xi,epsilion = 1):
        return [min(self.g[xi],self.rhs[xi]) + epsilion*self.h[xi],min(self.g[xi],self.rhs[xi])]

    def path(self):
        path = []
        x = self.xt
        start = self.x0
        ind = 0
        while x != start:
            j = x
            nei = self.CHILDREN[x]
            gset = [self.g[xi] for xi in nei]
            # collision check and make g Cost inf
            for i in range(len(nei)):
                if isCollide(self, nei[i],j)[0]:
                    gset[i] = np.inf
            parent = nei[np.argmin(gset)]
            path.append([x, parent])
            x = parent
            if ind > 100:
                break
            ind += 1
        return path

    #------------------Lifelong Plannning A* 
    def UpdateMembership(self, xi, xparent=None):
        if xi != self.x0:
            self.rhs[xi] = min([self.g[j] + self.getCOSTset(xi,j) for j in self.CHILDREN[xi]])
        self.OPEN.check_remove(xi)
        if self.g[xi] != self.rhs[xi]:
            self.OPEN.put(xi,self.key(xi))
    
    def ComputePath(self):
        print('computing path ...')
        while self.key(self.xt) > self.OPEN.top_key() or self.rhs[self.xt] != self.g[self.xt]:
            xi = self.OPEN.get()
            # if g > rhs, overconsistent
            if self.g[xi] > self.rhs[xi]: 
                self.g[xi] = self.rhs[xi]
                # add xi to expanded node set
                if xi not in self.CLOSED:
                    self.V.append(xi)
                self.CLOSED.add(xi)
            else: # underconsistent and consistent
                self.g[xi] = np.inf
                self.UpdateMembership(xi)
            for xj in self.CHILDREN[xi]:
                self.UpdateMembership(xj)

            # visualization(self)
            self.ind += 1
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(1)

    def change_env(self):
        _, _ = self.env.move_block(block_to_move=1,a = [0,2,0])
        self.done = False
        self.Path = []
        self.CLOSED = set()
        N = self.costset()
        for xi in N:
            self.UpdateMembership(xi)

if __name__ == '__main__':
    sta = time.time()
    Astar = Lifelong_Astar(1)
    Astar.ComputePath()
    for i in range(5):
        Astar.change_env()
        Astar.ComputePath()
        plt.pause(1)
    
    print(time.time() - sta)