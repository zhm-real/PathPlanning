#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Huiming Zhou
"""


import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

from queue import *
from mazemods import *


def depth_fist_search(xI, xG, n, m, O):
    q_dfs = QueueLIFO()
    q_dfs.put(xI)
    parent = {xI: xI}
    actions = {xI: (0, 0)}
    u_set = {(-1, 0), (1, 0), (0, 1), (0, -1)}

    while not q_dfs.empty():
        x_current = q_dfs.get()
        if x_current == xG:
            break
        for u_next in u_set:
            x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
            if 0 <= x_next[0] < n and 0 <= x_next[1] < m \
                    and x_next not in parent \
                    and not collisionCheck(x_current, u_next, O):
                q_dfs.put(x_next)
                parent[x_next] = x_current
                actions[x_next] = u_next
    [path_dfs, actions_dfs] = extractpath(xI, xG, parent, actions)
    [simple_cost, west_cost, east_cost] = cost_calculation(xI, actions_dfs, O)
    return path_dfs, actions_dfs, len(parent), simple_cost, west_cost, east_cost