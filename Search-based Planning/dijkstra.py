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


def DijkstraSearch(xI, xG, n, m, O, cost_type):
    q_dijk = QueuePrior()
    q_dijk.put(xI, 0)
    parent = {xI: xI}
    actions = {xI: (0, 0)}
    rec_cost = {xI: 0}
    u_set = {(-1, 0), (1, 0), (0, 1), (0, -1)}

    while not q_dijk.empty():
        x_current = q_dijk.get()
        if x_current == xG:
            break
        for u_next in u_set:
            x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
            if 0 <= x_next[0] < n and 0 <= x_next[1] < m \
                    and not collisionCheck(x_current, u_next, O):
                cost_x = costfunc(x_current, x_next, O, cost_type)
                new_cost = rec_cost[x_current] + cost_x
                if x_next not in rec_cost or new_cost < rec_cost[x_next]:
                    rec_cost[x_next] = new_cost
                    priority = new_cost
                    q_dijk.put(x_next, priority)
                    parent[x_next] = x_current
                    actions[x_next] = u_next
    [path_dijk, actions_dijk] = extractpath(xI, xG, parent, actions)
    [simple_cost, west_cost, east_cost] = cost_calculation(xI, actions_dijk, O)
    return path_dijk, actions_dijk, len(parent), simple_cost, west_cost, east_cost


# Cost function used in Dijkstra's algorithm
def costfunc(x_current, x_next, O, function_type):
    if function_type == "westcost":
        return x_next[0] ** 2
    elif function_type == "eastcost":
        maxX = 0
        for k in range(len(O)):
            westxO = O[k][1]
            if westxO > maxX:
                maxX = westxO
        return (maxX - x_next[0]) ** 2
    else:
        print("Please choose right cost function!")
