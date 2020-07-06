import numpy as np
import matplotlib.pyplot as plt

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search-based Planning/")
from Search_3D.env3D import env
from Search_3D import Astar3D
from Search_3D.utils3D import getDist, getRay
import pyrr

def StateSpace(env, factor = 0):
    boundary = env.boundary
    resolution = env.resolution
    xmin,xmax = boundary[0]+factor*resolution,boundary[3]-factor*resolution
    ymin,ymax = boundary[1]+factor*resolution,boundary[4]-factor*resolution
    zmin,zmax = boundary[2]+factor*resolution,boundary[5]-factor*resolution
    xarr = np.arange(xmin,xmax,resolution).astype(float)
    yarr = np.arange(ymin,ymax,resolution).astype(float)
    zarr = np.arange(zmin,zmax,resolution).astype(float)
    g = {}
    for x in xarr:
        for y in yarr:
            for z in zarr:
                g[(x,y,z)] = np.inf
    return g

def Heuristic(initparams,x):
    h = {}
    x = np.array(x)
    for xi in initparams.g.keys():
        h[xi] = max(abs(x-np.array(xi)))
    return h

def getNearest(Space,pt):
    '''get the nearest point on the grid'''
    mindis,minpt = 1000,None
    for pts in Space.keys(): 
        dis = getDist(pts,pt)
        if dis < mindis:
            mindis,minpt = dis,pts
    return minpt

class D_star(object):
    def __init__(self,resolution = 1):
        self.Alldirec = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
                                  [-1, 0, 0], [0, -1, 0], [0, 0, -1], [-1, -1, 0], [-1, 0, -1], [0, -1, -1],
                                  [-1, -1, -1],
                                  [1, -1, 0], [-1, 1, 0], [1, 0, -1], [-1, 0, 1], [0, 1, -1], [0, -1, 1],
                                  [1, -1, -1], [-1, 1, -1], [-1, -1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]])
        self.env = env(resolution = resolution)
        self.g = StateSpace(self.env)
        self.x0, self.xt = getNearest(self.g, self.env.start), getNearest(self.g, self.env.goal)
        self.h = Heuristic(self,self.x0) # getting heuristic for x0
        
if __name__ == '__main__':
    D = D_star(1)
    print(D.h[D.x0])