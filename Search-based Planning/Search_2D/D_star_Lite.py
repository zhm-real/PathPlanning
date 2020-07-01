"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class DStarLite:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.U = queue.QueuePrior()  # priority queue / U set
        self.g, self.rhs = {}, {}
        self.km = 0

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.xG] = 0
        self.U.put(self.xG, self.CalculateKey(self.xG))

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.xI, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def h(self, s_start, s):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s[0] - s_start[0]) + abs(s[1] - s_start[1])
        else:
            return math.hypot(s[0] - s_start[0], s[1] - s_start[1])

    def UpdateVertex(self, s):
        if s != self.xG:


    def getNeighbor(self, s):
        v_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                v_list.add(s_next)

        return v_list

    def getCost(self, s_start, s_end):

