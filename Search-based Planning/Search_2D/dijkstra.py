"""
Dijkstra 2D
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class Dijkstra:
    def __init__(self, s_start, s_goal):
        self.s_start, self.s_goal = s_start, s_goal

        self.Env = env.Env()
        self.plotting = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions                               # feasible input set
        self.obs = self.Env.obs                                     # position of obstacles

        self.g = {self.s_start: 0, self.s_goal: float("inf")}       # cost to come
        self.OPEN = queue.QueuePrior()                              # priority queue / U set
        self.OPEN.put(self.s_start, 0)
        self.CLOSED = []                                            # closed set & visited
        self.PARENT = {self.s_start: self.s_start}

    def searching(self):
        """
        Dijkstra Searching.
        :return: path, order of visited nodes in the planning
        """

        while self.OPEN:
            s = self.OPEN.get()

            if s == self.s_goal:                                    # stop condition
                break
            self.CLOSED.append(s)

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)
                if s_n not in self.g:
                    self.g[s_n] = float("inf")
                if new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.OPEN.put(s_n, new_cost)
                    self.PARENT[s_n] = s

        return self.extract_path(), self.CLOSED

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

    def extract_path(self):
        """
        Extract the path based on PARENT set.
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
    s_start = (5, 5)
    s_goal = (45, 25)

    dijkstra = Dijkstra(s_start, s_goal)
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = dijkstra.searching()
    plot.animation(path, visited, "Dijkstra's")                         # animation generate


if __name__ == '__main__':
    main()
