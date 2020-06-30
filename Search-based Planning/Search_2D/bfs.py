"""
BFS 2D (Breadth-first Searching)
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class BFS:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()
        self.plotting = plotting.Plotting(self.xI, self.xG)

        self.u_set = self.Env.motions                       # feasible input set
        self.obs = self.Env.obs                             # position of obstacles

        self.OPEN = queue.QueueFIFO()                       # OPEN set: visited nodes
        self.OPEN.put(self.xI)
        self.CLOSED = []                                    # CLOSED set: explored nodes
        self.PARENT = {self.xI: self.xI}                    # relations

    def searching(self):
        """
        :return: path, order of visited nodes in the planning
        """

        while not self.OPEN.empty():
            s = self.OPEN.get()
            if s == self.xG:
                break
            self.CLOSED.append(s)

            for u_next in self.u_set:                                       # explore neighborhoods
                s_next = tuple([s[i] + u_next[i] for i in range(2)])
                if s_next not in self.PARENT and s_next not in self.obs:    # node not visited and not in obstacles
                    self.OPEN.put(s_next)
                    self.PARENT[s_next] = s

        return self.extract_path(), self.CLOSED

    def extract_path(self):
        """
        Extract the path based on the relationship of nodes.
        :return: The planning path
        """

        path = [self.xG]
        s = self.xG

        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == self.xI:
                break

        return list(path)


def main():
    x_start = (5, 5)  # Starting node
    x_goal = (45, 25)  # Goal node

    bfs = BFS(x_start, x_goal)
    plot = plotting.Plotting(x_start, x_goal)
    fig_name = "Breadth-first Searching (BFS)"

    path, visited = bfs.searching()
    plot.animation(path, visited, fig_name)  # animation


if __name__ == '__main__':
    main()
