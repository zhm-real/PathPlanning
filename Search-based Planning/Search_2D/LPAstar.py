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
        self.U.put(self.xI, [self.h(self.xI), 0])

    def searching(self):
        self.computePath()
        path = self.extract_path()
        return path

    def computePath(self):
        while self.U.top_key() < self.CalculateKey(self.xG) \
                or self.rhs[self.xG] != self.g[self.xG]:
            s = self.U.get()
            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)

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
                u_min = min(u_min, self.g[x] + 1)
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

    @staticmethod
    def get_cost(x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: current input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    lpastar = LpaStar(x_start, x_goal, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)

    path = lpastar.searching()
    plot.plot_grid("test")
    px = [x[0] for x in path]
    py = [x[1] for x in path]
    plt.plot(px, py, color='red', marker='o')
    plt.show()


if __name__ == '__main__':
    main()
