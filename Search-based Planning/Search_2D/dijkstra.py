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
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()
        self.plotting = plotting.Plotting(self.xI, self.xG)

        self.u_set = self.Env.motions                               # feasible input set
        self.obs = self.Env.obs                                     # position of obstacles

        self.g = {self.xI: 0, self.xG: float("inf")}                # cost to come
        self.OPEN = queue.QueuePrior()                              # priority queue / OPEN set
        self.OPEN.put(self.xI, 0)
        self.CLOSED = []                                            # closed set & visited
        self.PARENT = {self.xI: self.xI}                            # relations

    def searching(self):
        """
        Searching using Dijkstra.
        :return: path, order of visited nodes in the planning
        """

        while not self.OPEN.empty():
            s = self.OPEN.get()
            if s == self.xG:                                        # stop condition
                break
            self.CLOSED.append(s)

            for u in self.u_set:                                    # explore neighborhoods
                s_next = tuple([s[i] + u[i] for i in range(2)])
                if s_next not in self.obs:                          # node not visited and not in obstacles
                    new_cost = self.g[s] + self.get_cost(s, u)
                    if s_next not in self.g:
                        self.g[s_next] = float("inf")
                    if new_cost < self.g[s_next]:
                        self.g[s_next] = new_cost
                        self.OPEN.put(s_next, new_cost)
                        self.PARENT[s_next] = s

        return self.extract_path(), self.CLOSED

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


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    dijkstra = Dijkstra(x_start, x_goal)
    plot = plotting.Plotting(x_start, x_goal)  # class Plotting

    fig_name = "Dijkstra's"
    path, visited = dijkstra.searching()
    plot.animation(path, visited, fig_name)  # animation generate


if __name__ == '__main__':
    main()
