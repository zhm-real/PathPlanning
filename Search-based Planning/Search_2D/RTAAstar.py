"""
RTAAstar 2D (Real-time Adaptive A*)
@author: huiming zhou
"""

import os
import sys
import copy
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class RtaAstar:
    def __init__(self, x_start, x_goal, N, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()

        self.u_set = self.Env.motions                   # feasible input set
        self.obs = self.Env.obs                         # position of obstacles

        self.N = N                                      # number of expand nodes each iteration
        self.visited = []                               # order of visited nodes in planning
        self.path = []                                  # path of each iteration

    def searching(self):
        s_start = self.xI                               # initialize start node

        while True:
            OPEN, CLOSED, g_table, PARENT = \
                self.Astar(s_start, self.N)

            if OPEN == "FOUND":                         # reach the goal node
                self.path.append(CLOSED)
                break

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)
            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)
            self.path.append(path_k)

    def cal_h_value(self, OPEN, CLOSED, g_table, PARENT):
        v_open = {}
        h_value = {}
        for (_, x) in OPEN.enumerate():
            v_open[x] = g_table[PARENT[x]] + 1 + self.h(x)
        s_open = min(v_open, key=v_open.get)
        f_min = min(v_open.values())
        for x in CLOSED:
            h_value[x] = f_min - g_table[x]

        return s_open, h_value

    def extract_path_in_CLOSE(self, s_end, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {}
            for u in self.u_set:
                s_next = tuple([s[i] + u[i] for i in range(2)])
                if s_next not in self.obs and s_next in h_value:
                    h_list[s_next] = h_value[s_next]
            s_key = max(h_list, key=h_list.get)                 # move to the smallest node with min h_value
            path.append(s_key)                                  # generate path
            s = s_key                                           # use end of this iteration as the start of next

            if s_key == s_end:                            # reach the expected node in OPEN set
                return s_start, list(reversed(path))

    def iteration(self, CLOSED):
        h_value = {}

        for s in CLOSED:
            h_value[s] = float("inf")                           # initialize h_value of CLOSED nodes

        while True:
            h_value_rec = copy.deepcopy(h_value)
            for s in CLOSED:
                h_list = []
                for u in self.u_set:
                    s_next = tuple([s[i] + u[i] for i in range(2)])
                    if s_next not in self.obs:
                        if s_next not in CLOSED:
                            h_list.append(self.get_cost(s, s_next) + self.h(s_next))
                        else:
                            h_list.append(self.get_cost(s, s_next) + h_value[s_next])
                h_value[s] = min(h_list)                        # update h_value of current node

            if h_value == h_value_rec:                          # h_value table converged
                return h_value

    def Astar(self, x_start, N):
        OPEN = queue.QueuePrior()                               # OPEN set
        OPEN.put(x_start, self.h(x_start))
        CLOSED = set()                                          # CLOSED set
        g_table = {x_start: 0, self.xG: float("inf")}           # cost to come
        PARENT = {x_start: x_start}                             # relations
        visited = []                                            # order of visited nodes
        count = 0                                               # counter

        while not OPEN.empty():
            count += 1
            s = OPEN.get()
            CLOSED.add(s)
            visited.append(s)

            if s == self.xG:                                                # reach the goal node
                self.visited.append(visited)
                return "FOUND", self.extract_path(x_start, PARENT), [], []

            for u in self.u_set:
                s_next = tuple([s[i] + u[i] for i in range(len(s))])
                if s_next not in self.obs and s_next not in CLOSED:
                    new_cost = g_table[s] + self.get_cost(s, u)
                    if s_next not in g_table:
                        g_table[s_next] = float("inf")
                    if new_cost < g_table[s_next]:                           # conditions for updating cost
                        g_table[s_next] = new_cost
                        PARENT[s_next] = s
                        OPEN.put(s_next, g_table[s_next] + self.h(s_next))

            if count == N:                                                   # expand needed CLOSED nodes
                break

        self.visited.append(visited)                                         # visited nodes in each iteration

        return OPEN, CLOSED, g_table, PARENT

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.xG]
        x_current = self.xG

        while True:
            x_current = parent[x_current]
            path_back.append(x_current)

            if x_current == x_start:
                break

        return list(reversed(path_back))

    def h(self, s):
        heuristic_type = self.heuristic_type
        goal = self.xG

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - s[0]) ** 2 + (goal[1] - s[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")

    @staticmethod
    def get_cost(x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1


def main():
    x_start = (10, 5)
    x_goal = (45, 25)

    rtaa = RtaAstar(x_start, x_goal, 220, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)
    fig_name = "Real-time Adaptive A* (RTAA*)"

    rtaa.searching()
    plot.animation_lrta(rtaa.path, rtaa.visited, fig_name)


if __name__ == '__main__':
    main()
