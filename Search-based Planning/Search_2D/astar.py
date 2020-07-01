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

        self.e = e  # weighted A*: e >= 1
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g = {self.xI: 0, self.xG: float("inf")}  # cost to come
        self.OPEN = queue.QueuePrior()  # priority queue / U set
        self.OPEN.put(self.xI, self.fvalue(self.xI))
        self.CLOSED = set()  # closed set & visited
        self.VISITED = []
        self.PARENT = {self.xI: self.xI}  # relations

    def searching(self):
        """
        Searching using A_star.

        :return: path, order of visited nodes in the planning
        """

        while not self.OPEN.empty():
            s = self.OPEN.get()
            self.CLOSED.add(s)
            self.VISITED.append(s)

            if s == self.xG:  # stop condition
                break

            for u in self.u_set:  # explore neighborhoods of current node
                s_next = tuple([s[i] + u[i] for i in range(2)])
                if s_next not in self.obs and s_next not in self.CLOSED:
                    new_cost = self.g[s] + self.get_cost(s, u)
                    if s_next not in self.g:
                        self.g[s_next] = float("inf")
                    if new_cost < self.g[s_next]:  # conditions for updating cost
                        self.g[s_next] = new_cost
                        self.PARENT[s_next] = s
                        self.OPEN.put(s_next, self.fvalue(s_next))

        return self.extract_path(self.PARENT), self.VISITED

    def repeated_Searching(self, xI, xG, e):
        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_Astar(xI, xG, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_Astar(self, xI, xG, e):
        g = {xI: 0, xG: float("inf")}
        OPEN = queue.QueuePrior()
        OPEN.put(xI, g[xI] + e * self.Heuristic(xI))
        CLOSED = set()
        PARENT = {xI: xI}
        VISITED = []

        while OPEN:
            s = OPEN.get()
            CLOSED.add(s)
            VISITED.append(s)

            if s == xG:
                break

            for u in self.u_set:  # explore neighborhoods of current node
                s_next = tuple([s[i] + u[i] for i in range(2)])
                if s_next not in self.obs and s_next not in CLOSED:
                    new_cost = g[s] + self.get_cost(s, u)
                    if s_next not in g:
                        g[s_next] = float("inf")
                    if new_cost < g[s_next]:  # conditions for updating cost
                        g[s_next] = new_cost
                        PARENT[s_next] = s
                        OPEN.put(s_next, g[s_next] + e * self.Heuristic(s_next))

        return self.extract_path(PARENT), VISITED

    def fvalue(self, x, e=1):
        """
        f = g + h. (g: cost to come, h: heuristic function)
        :param x: current state
        :return: f
        """

        return self.g[x] + e * self.Heuristic(x)

    def extract_path(self, PARENT):
        """
        Extract the path based on the relationship of nodes.

        :return: The planning path
        """

        path_back = [self.xG]
        x_current = self.xG

        while True:
            x_current = PARENT[x_current]
            path_back.append(x_current)

            if x_current == self.xI:
                break

        return list(path_back)

    @staticmethod
    def get_cost(x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: current input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1

    def Heuristic(self, state):
        """
        Calculate heuristic.

        :param state: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.xG  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    astar = Astar(x_start, x_goal, 1, "euclidean")  # weight e = 1
    plot = plotting.Plotting(x_start, x_goal)  # class Plotting

    fig_name = "A*"
    path, visited = astar.searching()
    plot.animation(path, visited, fig_name)  # animation generate

    # fig_name = "Repeated A*"
    # path, visited = astar.repeated_Searching(x_start, x_goal, 2.5)
    # plot.animation_ara_star(path, visited, fig_name)


if __name__ == '__main__':
    main()
