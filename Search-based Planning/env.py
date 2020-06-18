#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt

x_range, y_range = 51, 31                                   # size of background
motions = [(1, 0), (-1, 0), (0, 1), (0, -1)]                # feasible motion sets

def obs_map(xI, xG, name):
    """
    Initialize obstacles' positions

    :param xI: starting node
    :param xG: goal node
    :param name: title of figure
    :return: map of obstacles
    """

    obs_map = []
    for i in range(x_range):
        obs_map.append((i, 0))
    for i in range(x_range):
        obs_map.append((i, y_range-1))

    for i in range(y_range):
        obs_map.append((0, i))
    for i in range(y_range):
        obs_map.append((x_range-1, i))

    for i in range(10, 21):
        obs_map.append((i, 15))
    for i in range(15):
        obs_map.append((20, i))

    for i in range(15, 30):
        obs_map.append((30, i))
    for i in range(16):
        obs_map.append((40, i))

    obs_x = [obs_map[i][0] for i in range(len(obs_map))]
    obs_y = [obs_map[i][1] for i in range(len(obs_map))]

    plt.plot(xI[0], xI[1], "bs")
    plt.plot(xG[0], xG[1], "gs")
    plt.plot(obs_x, obs_y, "sk")
    plt.title(name, fontdict = None)
    plt.grid(True)
    plt.axis("equal")

    return obs_map

