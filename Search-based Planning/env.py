#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

x_range, y_range = 51, 31     # size of background

def obs_map():
    """
    Initialize obstacles' positions

    :return: map of obstacles
    """

    obs = []
    for i in range(x_range):
        obs.append((i, 0))
    for i in range(x_range):
        obs.append((i, y_range - 1))

    for i in range(y_range):
        obs.append((0, i))
    for i in range(y_range):
        obs.append((x_range - 1, i))

    for i in range(10, 21):
        obs.append((i, 15))
    for i in range(15):
        obs.append((20, i))

    for i in range(15, 30):
        obs.append((30, i))
    for i in range(16):
        obs.append((40, i))

    return obs
