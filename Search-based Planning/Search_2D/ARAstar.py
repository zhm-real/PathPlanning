"""
ARA_star 2D (Anytime Repairing A*)
@author: huiming zhou
"""

import os
import sys
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class AraStar:
    def __init__(self, s_start, s_goal, e, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()                                    # class Env

        self.u_set = self.Env.motions                           # feasible input set
        self.obs = self.Env.obs                                 # position of obstacles
        self.e = e                                              # initial weight
        self.g = {self.s_start: 0, self.s_goal: float("inf")}            # cost to come

        self.OPEN = queue.QueuePrior()                          # priority queue / U
        self.CLOSED = set()                                     # closed set
        self.INCONS = []                                        # incons set
        self.PARENT = {self.s_start: self.s_start}                        # relations
        self.path = []                                          # planning path
        self.visited = []                                       # order of visited nodes

    def searching(self):
        self.OPEN.put(self.s_start, self.fvalue(self.s_start))
        self.ImprovePath()
        self.path.append(self.extract_path())

        while self.update_e() > 1:                              # continue condition
            self.e -= 0.5                                       # increase weight
            OPEN_mid = [x for (p, x) in self.OPEN.enumerate()] + self.INCONS        # combine two sets
            self.OPEN = queue.QueuePrior()
            self.OPEN.put(self.s_start, self.fvalue(self.s_start))

            for x in OPEN_mid:
                self.OPEN.put(x, self.fvalue(x))                # update priority

            self.INCONS = []
            self.CLOSED = set()
            self.ImprovePath()                                  # improve path
            self.path.append(self.extract_path())

        return self.path, self.visited

    def ImprovePath(self):
        """
        :return: a e'-suboptimal path
        """

        visited_each = []

        while (self.fvalue(self.s_goal) >
               min([self.fvalue(x) for (p, x) in self.OPEN.enumerate()])):
            s = self.OPEN.get()

            if s not in self.CLOSED:
                self.CLOSED.add(s)

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)
                if s_n not in self.g or new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    visited_each.append(s_n)

                    if s_n not in self.CLOSED:
                        self.OPEN.put(s_n, self.fvalue(s_n))
                    else:
                        self.INCONS.append(s_n)

        self.visited.append(visited_each)

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.add(s_next)

        return s_list

    def update_e(self):
        c_OPEN, c_INCONS = float("inf"), float("inf")

        if self.OPEN:
            c_OPEN = min(self.g[x] +
                         self.Heuristic(x) for (p, x) in self.OPEN.enumerate())
        if self.INCONS:
            c_INCONS = min(self.g[x] +
                           self.Heuristic(x) for x in self.INCONS)
        if min(c_OPEN, c_INCONS) == float("inf"):
            return 1

        return min(self.e, self.g[self.s_goal] / min(c_OPEN, c_INCONS))

    def fvalue(self, x):
        return self.g[x] + self.e * self.Heuristic(x)

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = self.PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def Heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type                # heuristic type
        goal = self.s_goal                                  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    @staticmethod
    def cost(s_start, s_goal):
        """
        Calculate cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1


def main():
    x_start = (5, 5)  # Starting node
    x_goal = (45, 25)  # Goal node

    arastar = AraStar(x_start, x_goal, 2.5, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)

    fig_name = "Anytime Repairing A* (ARA*)"
    path, visited = arastar.searching()

    plot.animation_ara_star(path, visited, fig_name)


if __name__ == '__main__':
    main()
