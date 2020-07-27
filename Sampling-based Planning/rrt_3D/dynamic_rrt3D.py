"""
This is dynamic rrt code for 3D
@author: yue qi
"""
import numpy as np
from numpy.matlib import repmat
from collections import defaultdict
import time
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling-based Planning/")
from rrt_3D.env3D import env
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path, edgeset, isinbound
from rrt_3D.rrt3D import rrt

class dynamic_rrt_3D(rrt):
    
    def __init__(self):
        # variables in rrt
        self.env = env()
        self.Parent = {}
        self.E = edgeset() # edgeset
        self.V = [] # nodeset
        self.i = 0
        self.maxiter = 10000 # at least 2000 in this env
        self.stepsize = 0.5
        self.gamma = 500
        self.eta = 2*self.stepsize
        self.Path = []
        self.done = False
        self.x0 = tuple(self.env.goal)
        self.xt = tuple(self.env.start)
        # additional variables
        self.Flag = {}
        self.xrobot = tuple(self.env.start)
        
        self.V.append(self.x0)
        self.ind = 0
        self.fig = plt.figure(figsize=(10, 8))

    def RegrowRRT(self, xrobot):
        self.TrimRRT()
        self.GrowRRT(xrobot = xrobot)

    def TrimRRT(self):
        S = []
        i = 1
        for qi in self.V:
            if qi == self.x0:
                continue
            qp = self.Parent[qi]
            if qp == self.x0:
                continue
            if self.Flag[qp] == 'Invalid':
                self.Flag[qi] = 'Invalid'
                self.E.remove_edge([qi,qp]) # REMOVE edge that parent is invalid
            if self.Flag[qi] != 'Invalid':
                S.append(qi)
            i += 1
        self.V, self.E = self.CreateTreeFromNodes(S)
    
    def InvalidateNodes(self, obstacle, mode):
        E = self.FindAffectedEdges(obstacle, mode)
        for e in E:
            qe = self.ChildEndpointNode(e)
            self.Flag[qe] = 'Invalid'

    def Main(self):
        qgoal = tuple(self.env.start)
        qstart = tuple(self.env.goal)
        self.GrowRRT()
        self.done = True
        # visualization(self)
        self.done = False
        # change the enviroment
        new0,old0 = self.env.move_block(a=[0, 0, -2], s=0.5, block_to_move=1, mode='translation')
        while qgoal != qstart:
            qgoal = self.Parent[qgoal]
            # TODO move to qgoal and check for new obs
            xrobot = qstart
            # TODO if any new obstacle are observed
            self.InvalidateNodes(new0, mode = 'translation')
            for xi in self.Path:
                if self.Flag[tuple(xi[0])] == 'Invalid':
                    self.RegrowRRT(tuple(self.env.start))
        self.done = True
        self.Path, D = path(self)
        visualization(self)
        plt.show()
        
    def GrowRRT(self, Reversed = True, xrobot = None):
        # rrt.run()
        self.run(Reversed = Reversed, xrobot = xrobot)

    def CreateTreeFromNodes(self, S):
        return S, self.E

    def FindAffectedEdges(self, obstacle, mode):
        # if the nodes appears inside of the new range an obstacle moved, 
        # find the edges associated with this node. (the node is the parent)
        nodes = np.array(self.V)
        if mode == 'translation':
            affected_nodes = nodes[isinbound(obstacle, nodes, isarray = True)]
        elif mode == 'rotation':
            affected_nodes = nodes[isinbound(obstacle, nodes, mode = 'obb', isarray = True)]
        if len(affected_nodes) == 0:
            return []
        return self.E.get_edge(affected_nodes)

    def ChildEndpointNode(self, e):
        # if self.E.isEndNode(e[1]):
        return e[1]
        #else: 
        #    return self.ChildEndpointNode(e[1])

if __name__ == '__main__':
    rrt = dynamic_rrt_3D()
    rrt.Main()
