# this is the three dimensional configuration space for rrt
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np
# from utils3D import OBB2AABB

def R_matrix(z_angle,y_angle,x_angle):
    # x angle: row; y angle: pitch; z angle: yaw
    # generate rotation matrix in SO3
    # RzRyRx = R, ZYX intrinsic rotation
    # also (r1,r2,r3) in R3*3 in {W} frame
    # used in obb.O
    # [[R p]
    # [0T 1]] gives transformation from body to world 
    return np.array([[np.cos(z_angle), -np.sin(z_angle), 0.0], [np.sin(z_angle), np.cos(z_angle), 0.0], [0.0, 0.0, 1.0]])@ \
           np.array([[np.cos(y_angle), 0.0, np.sin(y_angle)], [0.0, 1.0, 0.0], [-np.sin(y_angle), 0.0, np.cos(y_angle)]])@ \
           np.array([[1.0, 0.0, 0.0], [0.0, np.cos(x_angle), -np.sin(x_angle)], [0.0, np.sin(x_angle), np.cos(x_angle)]])

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

class aabb(object):
    # make AABB out of blocks, 
    # P: center point
    # E: extents
    # O: Rotation matrix in SO(3), in {w}
    def __init__(self,AABB):
        self.P = [(AABB[3] + AABB[0])/2, (AABB[4] + AABB[1])/2, (AABB[5] + AABB[2])/2]# center point
        self.E = [(AABB[3] - AABB[0])/2, (AABB[4] - AABB[1])/2, (AABB[5] - AABB[2])/2]# extents
        self.O = [[1,0,0],[0,1,0],[0,0,1]]

class obb(object):
    # P: center point
    # E: extents
    # O: Rotation matrix in SO(3), in {w}
    def __init__(self, P, E, O):
        self.P = P
        self.E = E
        self.O = O

def getAABB2(blocks):
    # used in lineAABB
    AABB = []
    for i in blocks:
        AABB.append(aabb(i))
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
        self.AABB = getAABB2(self.blocks)
        self.AABB_pyrr = getAABB(self.blocks)
        self.balls = getballs()
        self.OBB = np.array([obb([2.6,2.5,1],[0.2,1,1],R_matrix(0,0,45))])
        #self.OBB = np.squeeze(np.vstack([self.OBB,OBB2AABB(self.OBB[0])]))
        #print(self.OBB)
        # self.OBB = []
        self.start = np.array([0.5, 2.5, 5.5])
        self.goal = np.array([19.0, 2.5, 5.5])
        self.t = 0 # time 

    def New_block(self):
        newblock = add_block()
        self.blocks = np.vstack([self.blocks,newblock])
        self.AABB = getAABB2(self.blocks)
        self.AABB_pyrr = getAABB(self.blocks)

    def move_start(self, x):
        self.start = x

    def move_block(self, a = [0,0,0], s = 0, v = [0.1,0,0], theta = [0,0,0], block_to_move = 0, obb_to_move = 0, mode = 'uniform'):
        # t is time , v is velocity in R3, a is acceleration in R3, s is increment ini time, 
        # R is an orthorgonal transform in R3*3, is the rotation matrix
        # (x',t') = (x + tv, t) is uniform transformation
        if mode == 'uniform':
            ori = np.array(self.blocks[block_to_move])
            self.blocks[block_to_move] = \
                np.array([ori[0] + self.t * v[0],\
                    ori[1] + self.t * v[1],\
                    ori[2] + self.t * v[2],\
                    ori[3] + self.t * v[0],\
                    ori[4] + self.t * v[1],\
                    ori[5] + self.t * v[2]])

            self.AABB[block_to_move].P = \
            [self.AABB[block_to_move].P[0] + self.t * v[0], \
            self.AABB[block_to_move].P[1] + self.t * v[1], \
            self.AABB[block_to_move].P[2] + self.t * v[2]]
            # return a range of block that the block might moved
            a = self.blocks[block_to_move]
            # return np.array([a[0] - self.resolution, a[1] - self.resolution, a[2] - self.resolution, \
            #                 a[3] + self.resolution, a[4] + self.resolution, a[5] + self.resolution]). \
                    # np.array([ori[0] - self.resolution, ori[1] - self.resolution, ori[2] - self.resolution, \
                    #         ori[3] + self.resolution, ori[4] + self.resolution, ori[5] + self.resolution])
            return a,ori
        # (x',t') = (x + a, t + s) is a translation
        if mode == 'translation':
            ori = np.array(self.blocks[block_to_move])
            self.blocks[block_to_move] = \
                np.array([ori[0] + a[0],\
                    ori[1] + a[1],\
                    ori[2] + a[2],\
                    ori[3] + a[0],\
                    ori[4] + a[1],\
                    ori[5] + a[2]])

            self.AABB[block_to_move].P = \
            [self.AABB[block_to_move].P[0] + a[0], \
            self.AABB[block_to_move].P[1] + a[1], \
            self.AABB[block_to_move].P[2] + a[2]]
            self.t += s
            # return a range of block that the block might moved
            a = self.blocks[block_to_move]
            return np.array([a[0] - self.resolution, a[1] - self.resolution, a[2] - self.resolution, \
                            a[3] + self.resolution, a[4] + self.resolution, a[5] + self.resolution]), \
                    np.array([ori[0] - self.resolution, ori[1] - self.resolution, ori[2] - self.resolution, \
                            ori[3] + self.resolution, ori[4] + self.resolution, ori[5] + self.resolution])
            # return a,ori
        # (x',t') = (Rx, t)
        if mode == 'rotation': # this makes an OBB rotate
            ori = [self.OBB[obb_to_move]]
            self.OBB[obb_to_move].O = R_matrix(z_angle=theta[0],y_angle=theta[1],x_angle=theta[2])
            return self.OBB[obb_to_move], ori[0]
          


if __name__ == '__main__':
    newenv = env()
