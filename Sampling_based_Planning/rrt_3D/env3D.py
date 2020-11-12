# this is the three dimensional configuration space for rrt
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np
# from utils3D import OBB2AABB

def R_matrix(z_angle,y_angle,x_angle):
    # s angle: row; y angle: pitch; z angle: yaw
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
    block = [[4.00e+00, 1.20e+01, 0.00e+00, 5.00e+00, 2.00e+01, 5.00e+00],
             [5.5e+00, 1.20e+01, 0.00e+00, 1.00e+01, 1.30e+01, 5.00e+00],
             [1.00e+01, 1.20e+01, 0.00e+00, 1.40e+01, 1.30e+01, 5.00e+00],
             [1.00e+01, 9.00e+00, 0.00e+00, 2.00e+01, 1.00e+01, 5.00e+00],
             [9.00e+00, 6.00e+00, 0.00e+00, 1.00e+01, 1.00e+01, 5.00e+00]]
    Obstacles = []
    for i in block:
        i = np.array(i)
        Obstacles.append([j for j in i])
    return np.array(Obstacles)

def getballs():
    spheres = [[2.0,6.0,2.5,1.0],[14.0,14.0,2.5,2]]
    Obstacles = []
    for i in spheres:
        Obstacles.append([j for j in i])
    return np.array(Obstacles)

def getAABB(blocks):
    # used for Pyrr package for detecting collision
    AABB = []
    for i in blocks:
        AABB.append(np.array([np.add(i[0:3], -0), np.add(i[3:6], 0)]))  # make AABBs alittle bit of larger
    return AABB

def getAABB2(blocks):
    # used in lineAABB
    AABB = []
    for i in blocks:
        AABB.append(aabb(i))
    return AABB

def add_block(block = [1.51e+01, 0.00e+00, 2.10e+00, 1.59e+01, 5.00e+00, 6.00e+00]):
    return block

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
        self.T = np.vstack([np.column_stack([self.O.T,-self.O.T@self.P]),[0,0,0,1]])

class env():
    def __init__(self, xmin=0, ymin=0, zmin=0, xmax=20, ymax=20, zmax=5, resolution=1):
    # def __init__(self, xmin=-5, ymin=0, zmin=-5, xmax=10, ymax=5, zmax=10, resolution=1):  
        self.resolution = resolution
        self.boundary = np.array([xmin, ymin, zmin, xmax, ymax, zmax]) 
        self.blocks = getblocks()
        self.AABB = getAABB2(self.blocks)
        self.AABB_pyrr = getAABB(self.blocks)
        self.balls = getballs()
        self.OBB = np.array([obb([5.0,7.0,2.5],[0.5,2.0,2.5],R_matrix(135,0,0)),
                             obb([12.0,4.0,2.5],[0.5,2.0,2.5],R_matrix(45,0,0))])
        self.start = np.array([2.0, 2.0, 2.0])
        self.goal = np.array([6.0, 16.0, 0.0])
        self.t = 0 # time 

    def New_block(self):
        newblock = add_block()
        self.blocks = np.vstack([self.blocks,newblock])
        self.AABB = getAABB2(self.blocks)
        self.AABB_pyrr = getAABB(self.blocks)

    def move_start(self, x):
        self.start = x

    def move_block(self, a = [0,0,0], s = 0, v = [0.1,0,0], block_to_move = 0, mode = 'translation'):
        # t is time , v is velocity in R3, a is acceleration in R3, s is increment ini time, 
        # R is an orthorgonal transform in R3*3, is the rotation matrix
        # (s',t') = (s + tv, t) is uniform transformation
        # (s',t') = (s + a, t + s) is a translation
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
        # (s',t') = (Rx, t)
    def move_OBB(self, obb_to_move = 0, theta=[0,0,0], translation=[0,0,0]):
    # theta stands for rotational angles around three principle axis in world frame
    # translation stands for translation in the world frame
        ori = [self.OBB[obb_to_move]]
        # move obb position
        self.OBB[obb_to_move].P = \
            [self.OBB[obb_to_move].P[0] + translation[0], 
            self.OBB[obb_to_move].P[1] + translation[1], 
            self.OBB[obb_to_move].P[2] + translation[2]]
        # Calculate orientation
        self.OBB[obb_to_move].O = R_matrix(z_angle=theta[0],y_angle=theta[1],x_angle=theta[2])
        # generating transformation matrix
        self.OBB[obb_to_move].T = np.vstack([np.column_stack([self.OBB[obb_to_move].O.T,\
            -self.OBB[obb_to_move].O.T@self.OBB[obb_to_move].P]),[translation[0],translation[1],translation[2],1]])
        return self.OBB[obb_to_move], ori[0]
          
if __name__ == '__main__':
    newenv = env()
