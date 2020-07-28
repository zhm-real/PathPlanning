"""
A_star 2D
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


class Astar:
    def __init__(self, start, goal, heuristic_type):
        self.s_start, self.s_goal = start, goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()                                        # class Env

        self.u_set = self.Env.motions                               # feasible input set
        self.obs = self.Env.obs                                     # position of obstacles

        self.g = {self.s_start: 0, self.s_goal: float("inf")}       # Cost to come
        self.OPEN = queue.QueuePrior()                              # priority queue / OPEN set
        self.OPEN.put(self.s_start, self.fvalue(self.s_start))
        self.CLOSED = []                                            # CLOSED set / VISITED order
        self.PARENT = {self.s_start: self.s_start}

    def searching(self):
        """
        A_star Searching.
        :return: path, order of visited nodes
        """

        while self.OPEN:
            s = self.OPEN.get()
            self.CLOSED.append(s)

            if s == self.s_goal:                                    # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)
                if s_n not in self.g:
                    self.g[s_n] = float("inf")
                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    self.OPEN.put(s_n, self.fvalue(s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def repeated_astar(self, e):
        """
        repeated a*.
        :param e: weight of a*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run a* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        OPEN = queue.QueuePrior()
        OPEN.put(s_start, g[s_start] + e * self.Heuristic(s_start))
        CLOSED = []
        PARENT = {s_start: s_start}

        while OPEN:
            s = OPEN.get()
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                if s_n not in CLOSED:
                    new_cost = g[s] + self.cost(s, s_n)
                    if s_n not in g:
                        g[s_n] = float("inf")
                    if new_cost < g[s_n]:                       # conditions for updating Cost
                        g[s_n] = new_cost
                        PARENT[s_n] = s
                        OPEN.put(s_n, g[s_n] + e * self.Heuristic(s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = []

        for u in self.u_set:
            s_list.append(tuple([s[i] + u[i] for i in range(2)]))

        return s_list

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def fvalue(self, x):
        """
        f = g + h. (g: Cost to come, h: heuristic function)
        :param x: current state
        :return: f
        """

        return self.g[x] + self.Heuristic(x)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
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

        heuristic_type = self.heuristic_type                    # heuristic type
        goal = self.s_goal                                      # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    astar = Astar(s_start, s_goal, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = astar.searching()
    plot.animation(path, visited, "A*")                         # animation

    # path, visited = astar.repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
