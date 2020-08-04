"""
This is dynamic rrt code for 3D
@author: yue qi
"""
import numpy as np
from numpy.matlib import repmat
from collections import defaultdict
import copy
import time
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling-based Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, cost, path, edgeset, isinbound, isinside
from rrt_3D.rrt3D import rrt
from rrt_3D.plot_util3D import make_get_proj, draw_block_list, draw_Spheres, draw_obb, draw_line, make_transparent

class dynamic_rrt_3D():
    
    def __init__(self):
        self.env = env()
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal)
        self.qrobot = self.x0
        self.current = tuple(self.env.start)
        self.stepsize = 0.25
        self.maxiter = 10000
        self.GoalProb = 0.05 # probability biased to the goal
        self.WayPointProb = 0.05 # probability falls back on to the way points

        self.V = [] # vertices
        self.Parent = {} # parent child relation
        self.Edge = set() # edge relation (node, parent node) tuple
        self.Path = []
        self.flag = {}# flag dictionary
        self.ind = 0
        self.i = 0

#--------Dynamic RRT algorithm
    def RegrowRRT(self):
        self.TrimRRT()
        self.GrowRRT()

    def TrimRRT(self):
        S = []
        i = 1
        print('trimming...')
        while i < len(self.V):
            qi = self.V[i]
            qp = self.Parent[qi]
            if self.flag[qp] == 'Invalid':
                self.flag[qi] = 'Invalid'
            if self.flag[qi] != 'Invalid':
                S.append(qi)
            i += 1
        self.CreateTreeFromNodes(S)
        print('trimming complete...')
    
    def InvalidateNodes(self, obstacle):
        Edges = self.FindAffectedEdges(obstacle)
        for edge in Edges:
            qe = self.ChildEndpointNode(edge)
            self.flag[qe] = 'Invalid'

#--------Extend RRT algorithm-----
    def initRRT(self):
        self.V.append(self.x0)
        self.flag[self.x0] = 'Valid'

    def GrowRRT(self):
        print('growing')
        qnew = self.x0
        tree = None
        distance_threshold = self.stepsize
        self.ind = 0
        while self.ind <= self.maxiter:
            qtarget = self.ChooseTarget()
            qnearest = self.Nearest(tree, qtarget)
            qnew, collide = self.Extend(qnearest, qtarget)
            if not collide:
                self.AddNode(qnearest, qnew)
                if getDist(qnew, self.xt) < distance_threshold:
                    self.AddNode(qnearest, self.xt)
                    self.flag[self.xt] = 'Valid'
                    break
                self.i += 1
            self.ind += 1
            # self.visualization()
        print('growing complete...')

    def ChooseTarget(self):
        # return the goal, or randomly choose a state in the waypoints based on probs
        p = np.random.uniform()
        if len(self.V) == 1:
            i = 0
        else:
            i = np.random.randint(0, high = len(self.V) - 1)
        if 0 < p < self.GoalProb:
            return self.xt
        elif self.GoalProb < p < self.GoalProb + self.WayPointProb:
            return self.V[i]
        elif self.GoalProb + self.WayPointProb < p < 1:
            return tuple(self.RandomState())
       
    def RandomState(self):
        # generate a random, obstacle free state
        xrand = sampleFree(self, bias=0)
        return xrand

    def AddNode(self, nearest, extended):
        self.V.append(extended)
        self.Parent[extended] = nearest
        self.Edge.add((extended, nearest))
        self.flag[extended] = 'Valid'

    def Nearest(self, tree, target):
        # TODO use kdTree to speed up search
        return nearest(self, target, isset=True)

    def Extend(self, nearest, target):
        extended, dist = steer(self, nearest, target, DIST = True)
        collide, _ = isCollide(self, nearest, target, dist)
        return extended, collide

#--------Main function
    def Main(self):
        # qstart = qgoal
        self.x0 = tuple(self.env.goal)
        # qgoal = qrobot
        self.xt = tuple(self.env.start)
        self.initRRT()
        self.GrowRRT()
        self.Path, D = path(self)
        self.done = True
        self.visualization() 
        plt.show()
        t = 0
        while True:
            # move the block while the robot is moving
            new, old = self.env.move_block(a=[0, 0, -0.2], mode='translation')
            self.InvalidateNodes(new)
            # if solution path contains invalid node
            self.done = True
            self.visualization()
            plt.show()
            invalid = self.PathisInvalid(self.Path)
            if invalid:
                self.done = False
                self.RegrowRRT()
                self.Path = []
                self.Path, D = path(self)
            
            if t == 8:
                break

#--------Additional utility functions
    def FindAffectedEdges(self, obstacle):
        # scan the graph for the changed edges in the tree.
        # return the end point and the affected
        Affectededges = []
        for e in self.Edge:
            child, parent = e
            collide, _ = isCollide(self, child, parent)
            if collide:
                Affectededges.append(e)
        return Affectededges

    def ChildEndpointNode(self, edge):
        return edge[0]

    def CreateTreeFromNodes(self, Nodes):
        self.V = []
        Parent = {}
        edges = set()
        for v in Nodes:
            self.V.append(v)
            Parent[v] = self.Parent[v]
            edges.add((v, Parent[v]))
        self.Parent = Parent
        self.Edge = edges

    def PathisInvalid(self, path):
        for edge in path:
            if self.flag[tuple(edge[0])] == 'Invalid' or self.flag[tuple(edge[1])] == 'Invalid':
                return True

    def path(self, Path=[], dist=0):
        x = self.xt
        while x != self.x0:
            x2 = self.Parent[x]
            Path.append(np.array([x, x2]))
            dist += getDist(x, x2)
            x = x2
        return Path, dist
            
#--------Visualization specialized for dynamic RRT
    def visualization(self):
        if self.ind % 100 == 0 or self.done:
            V = np.array(self.V)
            Path = np.array(self.Path)
            start = self.env.start
            goal = self.env.goal
            edges = []
            for i in self.Parent:
                edges.append([i,self.Parent[i]])
            ax = plt.subplot(111, projection='3d')
            # ax.view_init(elev=0.+ 0.03*initparams.ind/(2*np.pi), azim=90 + 0.03*initparams.ind/(2*np.pi))
            # ax.view_init(elev=0., azim=90.)
            ax.view_init(elev=8., azim=120.)
            ax.clear()
            # drawing objects
            draw_Spheres(ax, self.env.balls)
            draw_block_list(ax, self.env.blocks)
            if self.env.OBB is not None:
                draw_obb(ax, self.env.OBB)
            draw_block_list(ax, np.array([self.env.boundary]), alpha=0)
            draw_line(ax, edges, visibility=0.75, color='g')
            draw_line(ax, Path, color='r')
            # if len(V) > 0:
            #     ax.scatter3D(V[:, 0], V[:, 1], V[:, 2], s=2, color='g', )
            ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
            ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k')
            # adjust the aspect ratio
            xmin, xmax = self.env.boundary[0], self.env.boundary[3]
            ymin, ymax = self.env.boundary[1], self.env.boundary[4]
            zmin, zmax = self.env.boundary[2], self.env.boundary[5]
            dx, dy, dz = xmax - xmin, ymax - ymin, zmax - zmin
            ax.get_proj = make_get_proj(ax, 1 * dx, 1 * dy, 2 * dy)
            make_transparent(ax)
            #plt.xlabel('x')
            #plt.ylabel('y')
            ax.set_axis_off()
            plt.pause(0.0001)



if __name__ == '__main__':
    rrt = dynamic_rrt_3D()
    rrt.Main()
