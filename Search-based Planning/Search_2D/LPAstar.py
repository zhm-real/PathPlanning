"""
LPA_star 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class LpaStar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.U = queue.QueuePrior()  # priority queue / OPEN set
        self.g, self.rhs = {}, {}

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.xI] = 0
        self.U.put(self.xI, self.CalculateKey(self.xI))

    def searching(self):
        self.computePath()
        path = [self.extract_path()]

        obs_change = set()
        for j in range(14, 15):
            self.obs.add((30, j))
            obs_change.add((30, j))
        for s in obs_change:
            self.rhs[s] = float("inf")
            self.g[s] = float("inf")
            for x in self.get_neighbor(s):
                self.UpdateVertex(x)
        # for x in obs_change:
        #     self.obs.remove(x)
        # for x in obs_change:
        #     self.UpdateVertex(x)
        print(self.g[(29, 15)])
        print(self.g[(29, 14)])
        print(self.g[(29, 13)])
        print(self.g[(30, 13)])
        print(self.g[(31, 13)])
        print(self.g[(32, 13)])
        print(self.g[(33, 13)])
        print(self.g[(34, 13)])

        self.computePath()
        path.append(self.extract_path_test())

        return path, obs_change

    def computePath(self):
        while self.U.top_key() < self.CalculateKey(self.xG) \
                or self.rhs[self.xG] != self.g[self.xG]:
            s = self.U.get()
            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
            for x in self.get_neighbor(s):
                self.UpdateVertex(x)
        # return self.extract_path()

    def extract_path(self):
        path = []
        s = self.xG

        while True:
            g_list = {}
            for x in self.get_neighbor(s):
                g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            if s == self.xI:
                return list(reversed(path))
            path.append(s)

    def extract_path_test(self):
        path = []
        s = self.xG

        for k in range(30):
            g_list = {}
            for x in self.get_neighbor(s):
                g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
        return list(reversed(path))

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(s),
                min(self.g[s], self.rhs[s])]

    def UpdateVertex(self, u):
        if u != self.xI:
            u_min = float("inf")
            for x in self.get_neighbor(u):
                u_min = min(u_min, self.g[x] + self.get_cost(u, x))
            self.rhs[u] = u_min
        self.U.check_remove(u)
        if self.g[u] != self.rhs[u]:
            self.U.put(u, self.CalculateKey(u))

    def h(self, s):
        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.xG  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - s[0]) ** 2 + (goal[1] - s[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")

    def get_cost(self, s_start, s_end):
        """
        Calculate cost for this motion

        :param s_start:
        :param s_end:
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        if s_start not in self.obs:
            if s_end not in self.obs:
                return 1
            else:
                return float("inf")
        return float("inf")


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    lpastar = LpaStar(x_start, x_goal, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)

    path, obs = lpastar.searching()

    plot.plot_grid("Lifelong Planning A*")
    p = path[0]
    px = [x[0] for x in p]
    py = [x[1] for x in p]
    plt.plot(px, py, marker='o')
    plt.pause(0.5)

    p = path[1]
    px = [x[0] for x in p]
    py = [x[1] for x in p]
    plt.plot(px, py, marker='o')
    plt.pause(0.01)
    plt.show()


if __name__ == '__main__':
    main()
