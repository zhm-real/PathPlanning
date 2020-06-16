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

from bfs import *
from dijkstra import *
from astar import *


class Searching:
    def __init__(self, Start_State, Goal_State, n, m, O):
        self.xI = Start_State
        self.xG = Goal_State
        self.u_set = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        self.n = n
        self.m = m
        self.O = O

    def depth_fist_search(self):
        q_dfs = QueueLIFO()
        q_dfs.put(self.xI)
        parent = {self.xI: self.xI}
        actions = {self.xI: (0, 0)}

        while not q_dfs.empty():
            x_current = q_dfs.get()
            if x_current == xG:
                break
            for u_next in self.u_set:
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


if __name__ == '__main__':
    xI = (1, 1)
    xG = (23, 12)
    searching = Searching(xI, xG, n, m, O)

    [path_df, actions_df, num_visited_df, simple_cost_df, west_cost_df, east_cost_df] = searching.depth_fist_search()
    print('1 - Depth_First_Searching algorithm: ')
    print('Legal control actions: \n', actions_df)
    print('Number of explored nodes was [%d], basic cost was [%d], stay west cost was [%d], stay east cost was [%d] \n'
          % (num_visited_df, simple_cost_df, west_cost_df, east_cost_df))

    showPath(xI, xG, path_df, n, m, O, 'Depth First Searching algorithm')