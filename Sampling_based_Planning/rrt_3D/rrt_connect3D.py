# rrt connect algorithm
"""
This is rrt connect implementation for 3D
@author: yue qi
"""
import numpy as np
from numpy.matlib import repmat
from collections import defaultdict
import time
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling_based_Planning/")

from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path, edgeset
from rrt_3D.plot_util3D import set_axes_equal, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent


class Tree():
    def __init__(self, node):
        self.V = []
        self.Parent = {}
        self.V.append(node)
        # self.Parent[node] = None

    def add_vertex(self, node):
        if node not in self.V:
            self.V.append(node)
        
    def add_edge(self, parent, child):
        # here edge is defined a tuple of (parent, child) (qnear, qnew)
        self.Parent[child] = parent


class rrt_connect():
    def __init__(self):
        self.env = env()
        self.Parent = {}
        self.V = []
        self.E = set()
        self.i = 0
        self.maxiter = 10000
        self.stepsize = 0.5
        self.Path = []
        self.done = False
        self.qinit = tuple(self.env.start)
        self.qgoal = tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.qnew = None
        self.done = False
        
        self.ind = 0
        self.fig = plt.figure(figsize=(10, 8))


#----------Normal RRT algorithm
    def BUILD_RRT(self, qinit):
        tree = Tree(qinit)
        for k in range(self.maxiter):
            qrand = self.RANDOM_CONFIG()
            self.EXTEND(tree, qrand)
        return tree

    def EXTEND(self, tree, q):
        qnear = tuple(self.NEAREST_NEIGHBOR(q, tree))
        qnew, dist = steer(self, qnear, q)
        self.qnew = qnew # store qnew outside
        if self.NEW_CONFIG(q, qnear, qnew, dist=None):
            tree.add_vertex(qnew)
            tree.add_edge(qnear, qnew)
            if qnew == q:
                return 'Reached'
            else:
                return 'Advanced'
        return 'Trapped'

    def NEAREST_NEIGHBOR(self, q, tree):
        # find the nearest neighbor in the tree
        V = np.array(tree.V)
        if len(V) == 1:
            return V[0]
        xr = repmat(q, len(V), 1)
        dists = np.linalg.norm(xr - V, axis=1)
        return tuple(tree.V[np.argmin(dists)])

    def RANDOM_CONFIG(self):
        return tuple(sampleFree(self))

    def NEW_CONFIG(self, q, qnear, qnew, dist = None):
        # to check if the new configuration is valid or not by 
        # making a move is used for steer
        # check in bound
        collide, _ = isCollide(self, qnear, qnew, dist = dist)
        return not collide

#----------RRT connect algorithm
    def CONNECT(self, Tree, q):
        print('in connect')
        while True:
            S = self.EXTEND(Tree, q)
            if S != 'Advanced':
                break
        return S

    def RRT_CONNECT_PLANNER(self, qinit, qgoal):
        Tree_A = Tree(qinit)
        Tree_B = Tree(qgoal)
        for k in range(self.maxiter):
            print(k)
            qrand = self.RANDOM_CONFIG()
            if self.EXTEND(Tree_A, qrand) != 'Trapped':
                qnew = self.qnew # get qnew from outside
                if self.CONNECT(Tree_B, qnew) == 'Reached':
                    print('reached')
                    self.done = True
                    self.Path = self.PATH(Tree_A, Tree_B)
                    self.visualization(Tree_A, Tree_B, k)
                    plt.show()
                    return
                    # return
            Tree_A, Tree_B = self.SWAP(Tree_A, Tree_B)
            self.visualization(Tree_A, Tree_B, k)
        return 'Failure'

    # def PATH(self, tree_a, tree_b):
    def SWAP(self, tree_a, tree_b):
        tree_a, tree_b = tree_b, tree_a
        return tree_a, tree_b

    def PATH(self, tree_a, tree_b):
        qnew = self.qnew
        patha = []
        pathb = []
        while True:
            patha.append((tree_a.Parent[qnew], qnew))
            qnew = tree_a.Parent[qnew]
            if qnew == self.qinit or qnew == self.qgoal:
                break
        qnew = self.qnew
        while True:
            pathb.append((tree_b.Parent[qnew], qnew))
            qnew = tree_b.Parent[qnew]
            if qnew == self.qinit or qnew == self.qgoal:
                break
        return patha + pathb

#----------RRT connect algorithm        
    def visualization(self, tree_a, tree_b, index):
        if (index % 20 == 0 and index != 0) or self.done:
            # a_V = np.array(tree_a.V)
            # b_V = np.array(tree_b.V)
            Path = self.Path
            start = self.env.start
            goal = self.env.goal
            a_edges, b_edges = [], []
            for i in tree_a.Parent:
                a_edges.append([i,tree_a.Parent[i]])
            for i in tree_b.Parent:
                b_edges.append([i,tree_b.Parent[i]])
            ax = plt.subplot(111, projection='3d')
            ax.view_init(elev=90., azim=0.)
            ax.clear()
            draw_Spheres(ax, self.env.balls)
            draw_block_list(ax, self.env.blocks)
            if self.env.OBB is not None:
                draw_obb(ax, self.env.OBB)
            draw_block_list(ax, np.array([self.env.boundary]), alpha=0)
            draw_line(ax, a_edges, visibility=0.75, color='g')
            draw_line(ax, b_edges, visibility=0.75, color='y')
            draw_line(ax, Path, color='r')
            ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
            ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k')
            set_axes_equal(ax)
            make_transparent(ax)
            ax.set_axis_off()
            plt.pause(0.0001)



if __name__ == '__main__':
    p = rrt_connect()
    starttime = time.time()
    p.RRT_CONNECT_PLANNER(p.qinit, p.qgoal)
    print('time used = ' + str(time.time() - starttime))
