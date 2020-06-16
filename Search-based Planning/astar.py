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
from environment import *


def aStarSearch(xI, xG, n, m, O, heuristic_type):
    q_astar = QueuePrior()
    q_astar.put(xI, 0)
    parent = {xI: xI}
    actions = {xI: (0, 0)}
    rec_cost = {xI: 0}
    u_set = {(-1, 0), (1, 0), (0, 1), (0, -1)}

    while not q_astar.empty():
        x_current = q_astar.get()
        if x_current == xG:
            break
        for u_next in u_set:
            x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
            if 0 <= x_next[0] < n and 0 <= x_next[1] < m \
                    and not collisionCheck(x_current, u_next, O):
                new_cost = rec_cost[x_current] + 1
                if x_next not in rec_cost or new_cost < rec_cost[x_next]:
                    rec_cost[x_next] = new_cost
                    priority = new_cost + Heuristic(x_next, xG, heuristic_type)
                    q_astar.put(x_next, priority)
                    parent[x_next] = x_current
                    actions[x_next] = u_next
    [path_astar, actions_astar] = extractpath(xI, xG, parent, actions)
    [simple_cost, west_cost, east_cost] = cost_calculation(xI, actions_astar, O)
    return path_astar, actions_astar, len(parent), simple_cost, west_cost, east_cost


# Heuristic function used in A* algorithm
def Heuristic(state, goal, heuristic_type):
    if heuristic_type == "manhattanHeuristic":
        return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
    elif heuristic_type == "euclideanHeuristic":
        return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
    else:
        print("Please choose right heuristic type!")
