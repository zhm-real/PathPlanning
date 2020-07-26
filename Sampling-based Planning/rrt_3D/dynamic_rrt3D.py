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
from rrt_3D.utils3D import getDist, sampleFree, nearest, steer, isCollide, near, visualization, cost, path, edgeset

class dynamic_rrt_3D:
    
    def __init__(self):
        self.env = env()
        self.Parent = {}
        self.E = edgeset() # edgeset
        self.V = [] # nodeset
        self.i = 0
        self.maxiter = 2000 # at least 2000 in this env
        self.stepsize = 0.5
        self.gamma = 500
        self.eta = 2*self.stepsize
        self.Path = []
        self.done = False

    def RegrowRRT(self):
        self.TrimRRT()
        self.GrowRRT()

    def TrimRRT(self):
        S = set()
        i = 1
        for qi in self.V:
            qp = self.Parent(qi)
            if qp.flag == 'Invalid':
                qi.flag = 'Invalid'
            if qi.flag != 'Invalid':
                S.add(qi)
            i += 1
        self.V, self.E = self.CreateTreeFromNodes(S)
    
    def InvalidateNodes(self, obstacle):
        E = self.FindAffectedEdges(obstacle)
        for e in E:
            qe = self.ChildEndpointNode(e)
            qe.flag = 'Invalid'

        
    def GrowRRT(self):
        # TODO
        pass
            
    def CreateTreeFromNodes(self, S):
        #TODO
        pass

    def FindAffectedEdges(self, obstacle):
        #TODO
        pass

    def ChildEndpointNode(self):
        #TODO
        pass
