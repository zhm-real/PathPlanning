"""
LRTA_star 2D
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


class LrtAstar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g = {self.xI: 0, self.xG: float("inf")}
        self.OPEN = queue.QueuePrior()  # priority queue / OPEN
        self.OPEN.put(self.xI, self.h(self.xI))
        self.CLOSED = set()
        self.Parent = {self.xI: self.xI}

    def searching(self):
        h = {self.xI: self.h(self.xI)}
        s = self.xI
        parent = {self.xI: self.xI}
        visited = []
        count = 0
        while s != self.xG:
            count += 1
            print(count)
            visited.append(s)
            h_list = {}
            for u in self.u_set:
                s_next = tuple([s[i] + u[i] for i in range(len(s))])
                if s_next not in self.obs:
                    if s_next not in h:
                        h[s_next] = self.h(s_next)
                    h_list[s_next] = self.get_cost(s, s_next) + h[s_next]
            h_new = min(h_list.values())
            if h_new > h[s]:
                h[s] = h_new
            s_child = min(h_list, key=h_list.get)
            parent[s_child] = s
            s = s_child
        # path_get = self.extract_path(parent)
        return [], visited

    def extract_path(self, parent):
        path = [self.xG]
        s = self.xG

        while True:
            s = parent[s]
            path.append(s)

            if s == self.xI:
                break

        return path

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

    lrtastar = LrtAstar(x_start, x_goal, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)  # class Plotting

    path, visited = lrtastar.searching()
    pathx = [x[0] for x in path]
    pathy = [x[1] for x in path]
    vx = [x[0] for x in visited]
    vy = [x[1] for x in visited]
    plot.plot_grid("test")
    plt.plot(pathx, pathy, 'r')
    plt.plot(vx, vy, 'gray')
    plt.show()


if __name__ == '__main__':
    main()
