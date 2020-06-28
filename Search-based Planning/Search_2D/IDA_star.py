"""
IDA_Star 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import plotting
from Search_2D import env


class IdaStar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.visited = []

    def ida_star(self):
        bound = self.h(self.xI)
        path = [self.xI]

        while True:
            t = self.searching(path, 0, bound)
            if t == self.xG:
                return path, self.visited
            if t == float("inf"):
                return [], self.visited
            bound = t

    def searching(self, path, g, bound):
        s = path[-1]
        self.visited.append(s)
        f = g + self.h(s)

        if f > bound:
            return f
        if s == self.xG:
            return s

        res_min = float("inf")
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(len(s))])
            if s_next not in self.obs and s_next not in path:
                path.append(s_next)
                t = self.searching(path, g + 1, bound)
                if t == self.xG:
                    return self.xG
                if t < res_min:
                    res_min = t
                path.pop()

        return res_min

    def h(self, s):
        heuristic_type = self.heuristic_type
        goal = self.xG

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - s[0]) ** 2 + (goal[1] - s[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


def main():
    x_start = (5, 5)  # Starting node
    x_goal = (15, 20)  # Goal node

    ida_star = IdaStar(x_start, x_goal, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)

    path, visited = ida_star.ida_star()
    print(len(visited))

    if path:
        plot.plot_grid("IDA_star")
        plot.plot_path(visited, 'gray', True)
        plot.plot_path(path)
        plt.show()
    else:
        print("Path not found!")
    plot.plot_grid("IDA")


if __name__ == '__main__':
    main()
