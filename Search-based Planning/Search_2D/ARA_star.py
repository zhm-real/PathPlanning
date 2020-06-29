"""
ARA_star 2D (Anytime Repairing A*)
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class AraStar:
    def __init__(self, x_start, x_goal, e, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()                                    # class Env

        self.u_set = self.Env.motions                           # feasible input set
        self.obs = self.Env.obs                                 # position of obstacles
        self.e = e                                              # initial weight
        self.g = {self.xI: 0, self.xG: float("inf")}            # cost to come

        self.OPEN = queue.QueuePrior()                          # priority queue / OPEN
        self.CLOSED = set()                                     # closed set
        self.INCONS = []                                        # incons set
        self.PARENT = {self.xI: self.xI}                        # relations
        self.path = []                                          # planning path
        self.visited = []                                       # order of visited nodes

    def searching(self):
        self.OPEN.put(self.xI, self.fvalue(self.xI))
        self.ImprovePath()
        self.path.append(self.extract_path())

        while self.update_e() > 1:                              # continue condition
            self.e -= 0.5                                       # increase weight
            OPEN_mid = [x for (p, x) in self.OPEN.enumerate()] + self.INCONS        # combine two sets
            self.OPEN = queue.QueuePrior()
            self.OPEN.put(self.xI, self.fvalue(self.xI))

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

        while (self.fvalue(self.xG) >
               min([self.fvalue(x) for (p, x) in self.OPEN.enumerate()])):
            s = self.OPEN.get()

            if s not in self.CLOSED:
                self.CLOSED.add(s)

            for u_next in self.u_set:
                s_next = tuple([s[i] + u_next[i] for i in range(len(s))])
                if s_next not in self.obs:
                    new_cost = self.g[s] + self.get_cost(s, u_next)
                    if s_next not in self.g or new_cost < self.g[s_next]:
                        self.g[s_next] = new_cost
                        self.PARENT[s_next] = s
                        visited_each.append(s_next)

                        if s_next not in self.CLOSED:
                            self.OPEN.put(s_next, self.fvalue(s_next))
                        else:
                            self.INCONS.append(s_next)

        self.visited.append(visited_each)

    def update_e(self):
        c_OPEN, c_INCONS = float("inf"), float("inf")

        if self.OPEN:
            c_OPEN = min(self.g[x] + self.Heuristic(x) for (p, x) in self.OPEN.enumerate())

        if self.INCONS:
            c_INCONS = min(self.g[x] + self.Heuristic(x) for x in self.INCONS)

        if min(c_OPEN, c_INCONS) == float("inf"):
            return 1

        return min(self.e, self.g[self.xG] / min(c_OPEN, c_INCONS))

    def fvalue(self, x):
        return self.g[x] + self.e * self.Heuristic(x)

    def extract_path(self):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.xG]
        x_current = self.xG

        while True:
            x_current = self.PARENT[x_current]
            path_back.append(x_current)

            if x_current == self.xI:
                break

        return list(path_back)

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

    def Heuristic(self, state):
        """
        Calculate heuristic.
        :param state: current node (state)
        :return: heuristic
        """

        heuristic_type = self.heuristic_type
        goal = self.xG

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


def main():
    x_start = (5, 5)  # Starting node
    x_goal = (49, 5)  # Goal node

    arastar = AraStar(x_start, x_goal, 2.5, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)

    fig_name = "Anytime Repairing A* (ARA*)"
    path, visited = arastar.searching()

    plot.animation_ara_star(path, visited, fig_name)


if __name__ == '__main__':
    main()
