#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import numpy as np

col, row = 50, 30                                               # size of background
motions = [(1, 0), (-1, 0), (0, 1), (0, -1)]                    # feasible motion sets


def obstacles():
    """
    Design the obstacles' positions.
    :return: the map of obstacles.
    """

    background = [[[1., 1., 1.]
                   for x in range(col)] for y in range(row)]
    for j in range(col):
        background[0][j] = [0., 0., 0.]
        background[row - 1][j] = [0., 0., 0.]
    for i in range(row):
        background[i][0] = [0., 0., 0.]
        background[i][col - 1] = [0., 0., 0.]
    for i in range(10, 20):
        background[15][i] = [0., 0., 0.]
    for i in range(15):
        background[row - 1 - i][30] = [0., 0., 0.]
        background[i + 1][20] = [0., 0., 0.]
        background[i + 1][40] = [0., 0., 0.]
    return background


def map_obs():
    """
    Using a matrix to represent the position of obstacles,
    which is used for obstacle detection.
    :return: a matrix, in which '1' represents obstacle.
    """

    obs_map = np.zeros((col, row))
    pos_map = obstacles()
    for i in range(col):
        for j in range(row):
            if pos_map[j][i] == [0., 0., 0.]:
                obs_map[i][j] = 1
    return obs_map