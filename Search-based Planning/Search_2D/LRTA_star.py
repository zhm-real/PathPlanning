"""
LRTA_star_N 2D
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


class LrtAstarN:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.N = 150
        self.visited = []

    def searching(self):
        s_start = self.xI

        path = []
        count = 0

        while True:
            # if count == 2:
            #     return path
            # count += 1

            h_table = {}
            OPEN, CLOSED = self.Astar(s_start, self.N)

            if OPEN == "end":
                path.append(CLOSED)
                return path

            for x in CLOSED:
                h_table[x] = 2000

            while True:
                h_table_rec = copy.deepcopy(h_table)
                for s in CLOSED:
                    h_list = []
                    for u in self.u_set:
                        s_next = tuple([s[i] + u[i] for i in range(2)])
                        if s_next not in self.obs:
                            if s_next not in CLOSED:
                                h_list.append(self.get_cost(s, s_next) + self.h(s_next))
                            else:
                                h_list.append(self.get_cost(s, s_next) + h_table[s_next])
                    h_table[s] = min(h_list)
                if h_table == h_table_rec:
                    break

            path_k = [s_start]
            x = s_start
            while True:
                h_xlist = {}
                for u in self.u_set:
                    x_next = tuple([x[i] + u[i] for i in range(2)])
                    if x_next not in self.obs:
                        if x_next in CLOSED:
                            h_xlist[x_next] = h_table[x_next]
                        else:
                            h_xlist[x_next] = self.h(x_next)
                s_key = min(h_xlist, key=h_xlist.get)
                path_k.append(s_key)
                x = s_key
                if s_key not in CLOSED:
                    break
            s_start = path_k[-1]

            path.append(path_k)

    def Astar(self, x_start, N):
        OPEN = queue.QueuePrior()
        OPEN.put(x_start, self.h(x_start))
        CLOSED = set()
        g_table = {x_start: 0, self.xG: float("inf")}
        parent = {x_start: x_start}
        count = 0
        visited = []

        while not OPEN.empty():
            count += 1
            s = OPEN.get()
            CLOSED.add(s)
            visited.append(s)
            if s == self.xG:
                path = self.extract_path(x_start, parent)
                self.visited.append(visited)
                return "end", path

            for u in self.u_set:
                s_next = tuple([s[i] + u[i] for i in range(len(s))])
                if s_next not in self.obs and s_next not in CLOSED:
                    new_cost = g_table[s] + self.get_cost(s, u)
                    if s_next not in g_table:
                        g_table[s_next] = float("inf")
                    if new_cost < g_table[s_next]:  # conditions for updating cost
                        g_table[s_next] = new_cost
                        parent[s_next] = s
                        OPEN.put(s_next, g_table[s_next] + self.h(s_next))

            if count == N:
                break
        self.visited.append(visited)

        return OPEN, CLOSED

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
    x_start = (10, 5)  # Starting node
    x_goal = (45, 25)  # Goal node

    lrtastarn = LrtAstarN(x_start, x_goal, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)

    path = lrtastarn.searching()
    plot.plot_grid("LRTA_star_N")

    for k in range(len(path)):
        plot.plot_visited(lrtastarn.visited[k])
        plt.pause(0.5)
        plot.plot_path(path[k])
        plt.pause(0.5)
    plt.pause(0.5)

    path_u = []
    for i in range(len(path)):
        for j in range(len(path[i])):
            path_u.append(path[i][j])
    plot.plot_path(path_u)
    plt.pause(0.2)
    plt.show()


if __name__ == '__main__':
    main()
