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

        self.OPEN = queue.QueuePrior()  # priority queue / U set
        self.g, self.v = {}, {}

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.v[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.v[self.xI] = 0
        self.OPEN.put(self.xI, self.Key(self.xI))
        self.CLOSED = set()

    def searching(self):
        self.ComputePath()
        path = [self.extract_path()]
        # self.print_g()

        obs_change = set()
        for i in range(25, 30):
            self.obs.add((i, 15))
            obs_change.add((i, 15))

        self.obs.add((30, 14))
        obs_change.add((30, 14))

        for s in obs_change:
            self.v[s] = float("inf")
            self.g[s] = float("inf")
            for x in self.get_neighbor(s):
                self.UpdateMembership(x)

        # for x in obs_change:
        #     self.obs.remove(x)
        # for x in obs_change:
        #     self.UpdateVertex(x)

        self.ComputePath()
        path.append(self.extract_path_test())
        self.print_g()

        return path, obs_change

    def ComputePath(self):
        while self.Key(self.xG) > self.OPEN.top_key() \
                or self.v[self.xG] < self.g[self.xG]:
            s = self.OPEN.get()
            if self.v[s] > self.g[s]:
                self.v[s] = self.g[s]
                self.CLOSED.add(s)



        while self.OPEN.top_key() < self.Key(self.xG) \
                or self.v[self.xG] != self.g[self.xG]:
            s = self.OPEN.get()
            if self.g[s] > self.v[s]:
                self.g[s] = self.v[s]
            else:
                self.g[s] = float("inf")
                self.UpdateMembership(s)
            for x in self.get_neighbor(s):
                self.UpdateMembership(x)
        # return self.extract_path()

    def UpdateMembership(self, s):
        if self.v[s] != self.g[s]:
            if s not in self.CLOSED:
                self.OPEN.put(s, self.Key(s))
        else:
            if s in self.OPEN:
                self.OPEN.remove(s)

    def print_g(self):
        print("he")
        for k in range(self.Env.y_range):
            j = self.Env.y_range - k - 1
            string = ""
            for i in range(self.Env.x_range):
                if self.g[(i, j)] == float("inf"):
                    string += ("00" + ', ')
                else:
                    if self.g[(i, j)] // 10 == 0:
                        string += ("0" + str(self.g[(i, j)]) + ', ')
                    else:
                        string += (str(self.g[(i, j)]) + ', ')
            print(string)

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

        for k in range(70):
            g_list = {}
            for x in self.get_neighbor(s):
                g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            if s == self.xI:
                return list(reversed(path))
            path.append(s)
        return list(reversed(path))

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def Key(self, s):
        return [min(self.g[s], self.v[s]) + self.h(s),
                min(self.g[s], self.v[s])]

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

        # if s_start not in self.obs:
        #     if s_end not in self.obs:
        #         return 1
        #     else:
        #         return float("inf")
        # return float("inf")
        return 1

def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    lpastar = LpaStar(x_start, x_goal, "manhattan")
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
