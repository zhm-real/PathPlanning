# this is the three dimensional configuration space for rrt
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np


def getblocks():
    # AABBs
    block = [[3.10e+00, 0.00e+00, 2.10e+00, 3.90e+00, 5.00e+00, 6.00e+00],
             [9.10e+00, 0.00e+00, 2.10e+00, 9.90e+00, 5.00e+00, 6.00e+00],
             #[1.51e+01, 0.00e+00, 2.10e+00, 1.59e+01, 5.00e+00, 6.00e+00],
             #[1.00e-01, 0.00e+00, 0.00e+00, 9.00e-01, 5.00e+00, 3.90e+00],
             #[6.10e+00, 0.00e+00, 0.00e+00, 6.90e+00, 5.00e+00, 3.90e+00],
             [1.21e+01, 0.00e+00, 0.00e+00, 1.29e+01, 5.00e+00, 3.90e+00],
             [1.81e+01, 0.00e+00, 0.00e+00, 1.89e+01, 5.00e+00, 3.90e+00]]
    Obstacles = []
    for i in block:
        i = np.array(i)
        Obstacles.append([j for j in i])
    return np.array(Obstacles)

def getAABB(blocks):
    # used for Pyrr package for detecting collision
    AABB = []
    for i in blocks:
        AABB.append(np.array([np.add(i[0:3], -0), np.add(i[3:6], 0)]))  # make AABBs alittle bit of larger
    return AABB

def getballs():
    spheres = [[16,2.5,4,2],[10,2.5,1,1]]
    Obstacles = []
    for i in spheres:
        Obstacles.append([j for j in i])
    return np.array(Obstacles)

def add_block(block = [1.51e+01, 0.00e+00, 2.10e+00, 1.59e+01, 5.00e+00, 6.00e+00]):
    return block

class env():
    def __init__(self, xmin=0, ymin=0, zmin=0, xmax=20, ymax=5, zmax=6, resolution=1):
        self.resolution = resolution
        self.boundary = np.array([xmin, ymin, zmin, xmax, ymax, zmax]) 
        self.blocks = getblocks()
        self.AABB = getAABB(self.blocks)
        self.balls = getballs()
        self.start = np.array([0.5, 2.5, 5.5])
        self.goal = np.array([19.0, 2.5, 5.5])

    def change(self):
        newblock = add_block()
        self.blocks = np.vstack([self.blocks,newblock])
        self.AABB = getAABB(self.blocks)

if __name__ == '__main__':
    newenv = env()
    print(newenv.balls)
