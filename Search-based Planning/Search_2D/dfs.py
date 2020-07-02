"""
Depth-first Searching_2D (DFS)
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class DFS:
    def __init__(self, s_start, s_goal):
        self.s_start, self.s_goal = s_start, s_goal

        self.Env = env.Env()
        self.plotting = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions                           # feasible input set
        self.obs = self.Env.obs                                 # position of obstacles

        self.OPEN = queue.QueueLIFO()                           # OPEN set
        self.OPEN.put(self.s_start)
        self.CLOSED = []                                        # CLOSED set / visited order
        self.PARENT = {self.s_start: self.s_start}

    def searching(self):
        """
        Depth-first Searching
        :return: planning path, visited order
        """

        while self.OPEN:
            s = self.OPEN.get()

            if s == self.s_goal:
                break
            self.CLOSED.append(s)

            for s_n in self.get_neighbor(s):
                if s_n not in self.PARENT:                      # node not explored
                    self.OPEN.put(s_n)
                    self.PARENT[s_n] = s

        return self.extract_path(), self.CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = []

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.append(s_next)

        return s_list

    def extract_path(self):
        """
        Extract the path based on the relationship of nodes.
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


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    dfs = DFS(s_start, s_goal)
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = dfs.searching()
    plot.animation(path, visited, "Depth-first Searching (DFS)")  # animation


if __name__ == '__main__':
    main()
