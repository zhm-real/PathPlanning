"""
Best-First Searching
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_based_Planning.Search_2D import plotting, env


class BestFirst:
    def __init__(self, s_start, s_goal):
        self.s_start = s_start
        self.s_goal = s_goal

        self.Env = env.Env()
        self.plotting = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # OPEN set: visited nodes
        self.CLOSED = []  # CLOSED set / visited order
        self.PARENT = dict()  # recorded parent

    def searching(self):
        """
        Best-first Searching
        :return: planning path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        heapq.heappush(self.OPEN,
                       (self.heuristic(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)

            if s == self.s_goal:
                break
            self.CLOSED.append(s)

            for s_n in self.get_neighbor(s):
                if self.is_collision(s, s_n):
                    continue

                if s_n not in self.PARENT:  # node not explored
                    heapq.heappush(self.OPEN, (self.heuristic(s_n), s_n))
                    self.PARENT[s_n] = s

        return self.extract_path(), self.CLOSED

    def heuristic(self, s):
        """
        estimated distance between current state and goal state.
        :param s: current state
        :return: Euclidean distance
        """

        return math.hypot(s[0] - self.s_goal[0], s[1] - self.s_goal[1])

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

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

    BF = BestFirst(s_start, s_goal)
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = BF.searching()
    plot.animation(path, visited, "Best-first Searching")  # animation


if __name__ == '__main__':
    main()
