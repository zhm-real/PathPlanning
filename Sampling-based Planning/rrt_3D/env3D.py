# this is the three dimensional configuration space for rrt
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""
import numpy as np


def getblocks(resolution):
    # AABBs
    block = [[3.10e+00, 0.00e+00, 2.10e+00, 3.90e+00, 5.00e+00, 6.00e+00],
             [9.10e+00, 0.00e+00, 2.10e+00, 9.90e+00, 5.00e+00, 6.00e+00],
             [1.51e+01, 0.00e+00, 2.10e+00, 1.59e+01, 5.00e+00, 6.00e+00],
             [1.00e-01, 0.00e+00, 0.00e+00, 9.00e-01, 5.00e+00, 3.90e+00],
             [6.10e+00, 0.00e+00, 0.00e+00, 6.90e+00, 5.00e+00, 3.90e+00],
             [1.21e+01, 0.00e+00, 0.00e+00, 1.29e+01, 5.00e+00, 3.90e+00],
             [1.81e+01, 0.00e+00, 0.00e+00, 1.89e+01, 5.00e+00, 3.90e+00]]
    Obstacles = []
    for i in block:
        i = np.array(i)
        Obstacles.append((i[0] / resolution, i[1] / resolution, i[2] / resolution, i[3] / resolution, i[4] / resolution,
                          i[5] / resolution))
    return np.array(Obstacles)


class env():
    def __init__(self, xmin=0, ymin=0, zmin=0, xmax=20, ymax=5, zmax=6, resolution=1):
        self.resolution = resolution
        self.boundary = np.array([xmin, ymin, zmin, xmax, ymax, zmax]) / resolution
        self.blocks = getblocks(resolution)
        self.start = np.array([0.5, 2.5, 5.5])
        self.goal = np.array([19.0, 2.5, 5.5])

    def visualize(self):
        # fig = plt.figure()
        # TODO: do visualizations
        return


if __name__ == '__main__':
    newenv = env()
    X = StateSpace(newenv.boundary, newenv.resolution)
    print(X)
