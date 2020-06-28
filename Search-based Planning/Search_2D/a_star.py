"""
A_star 2D
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class Astar:
    def __init__(self, x_start, x_goal, e, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.e = e
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g = {self.xI: 0, self.xG: float("inf")}
        self.OPEN = queue.QueuePrior()  # priority queue / OPEN
        self.OPEN.put(self.xI, self.fvalue(self.xI))
        self.CLOSED = []
        self.Parent = {self.xI: self.xI}

    def searching(self):
        """
        Searching using A_star.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        while not self.OPEN.empty():
            s = self.OPEN.get()
            self.CLOSED.append(s)

            if s == self.xG:  # stop condition
                break

            for u_next in self.u_set:  # explore neighborhoods of current node
                s_next = tuple([s[i] + u_next[i] for i in range(len(s))])
                if s_next not in self.obs and s_next not in self.CLOSED:
                    new_cost = self.g[s] + self.get_cost(s, u_next)
                    if s_next not in self.g:
                        self.g[s_next] = float("inf")
                    if new_cost < self.g[s_next]:  # conditions for updating cost
                        self.g[s_next] = new_cost
                        self.Parent[s_next] = s
                        self.OPEN.put(s_next, self.fvalue(s_next))

        return self.extract_path(), self.CLOSED

    def fvalue(self, x):
        h = self.e * self.Heuristic(x)
        return self.g[x] + h

    def extract_path(self):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.xG]
        x_current = self.xG

        while True:
            x_current = self.Parent[x_current]
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
        :param goal: goal node (state)
        :param heuristic_type: choosing different heuristic functions
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
    x_goal = (49, 25)  # Goal node

    astar = Astar(x_start, x_goal, 1, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)  # class Plotting

    fig_name = "A* Algorithm"
    path, visited = astar.searching()
    plot.animation(path, visited, fig_name)  # animation generate


if __name__ == '__main__':
    main()
