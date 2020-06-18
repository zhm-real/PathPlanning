#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt

x_range, y_range = 51, 31                                   # size of background

def obs_map():
    """
    Initialize obstacles' positions

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

    return obs_map


def show_map(xI, xG, obs_map, name):
    obs_x = [obs_map[i][0] for i in range(len(obs_map))]
    obs_y = [obs_map[i][1] for i in range(len(obs_map))]

    plt.plot(xI[0], xI[1], "bs")
    plt.plot(xG[0], xG[1], "gs")
    plt.plot(obs_x, obs_y, "sk")
    plt.title(name, fontdict=None)
    plt.grid(True)
    plt.axis("equal")
