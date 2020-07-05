"""
Bidirectional_a_star 2D
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


class BidirectionalAstar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()                                                # class Env

        self.u_set = self.Env.motions                                       # feasible input set
        self.obs = self.Env.obs                                             # position of obstacles

        self.g_fore = {self.s_start: 0, self.s_goal: float("inf")}          # cost to come: from s_start
        self.g_back = {self.s_goal: 0, self.s_start: float("inf")}          # cost to come: form s_goal

        self.OPEN_fore = queue.QueuePrior()                                 # OPEN set for foreward searching
        self.OPEN_fore.put(self.s_start,
                           self.g_fore[self.s_start] + self.h(self.s_start, self.s_goal))
        self.OPEN_back = queue.QueuePrior()                                 # OPEN set for backward searching
        self.OPEN_back.put(self.s_goal,
                           self.g_back[self.s_goal] + self.h(self.s_goal, self.s_start))

        self.CLOSED_fore = []                                               # CLOSED set for foreward
        self.CLOSED_back = []                                               # CLOSED set for backward

        self.PARENT_fore = {self.s_start: self.s_start}
        self.PARENT_back = {self.s_goal: self.s_goal}

    def searching(self):
        s_meet = self.s_start

        while self.OPEN_fore and self.OPEN_back:
            # solve foreward-search
            s_fore = self.OPEN_fore.get()

            if s_fore in self.PARENT_back:
                s_meet = s_fore
                break
            self.CLOSED_fore.append(s_fore)

            for s_n in self.get_neighbor(s_fore):
                new_cost = self.g_fore[s_fore] + self.cost(s_fore, s_n)
                if s_n not in self.g_fore:
                    self.g_fore[s_n] = float("inf")
                if new_cost < self.g_fore[s_n]:
                    self.g_fore[s_n] = new_cost
                    self.PARENT_fore[s_n] = s_fore
                    self.OPEN_fore.put(s_n, new_cost + self.h(s_n, self.s_goal))

            # solve backward-search
            s_back = self.OPEN_back.get()
            if s_back in self.PARENT_fore:
                s_meet = s_back
                break
            self.CLOSED_back.append(s_back)

            for s_n in self.get_neighbor(s_back):
                new_cost = self.g_back[s_back] + self.cost(s_back, s_n)
                if s_n not in self.g_back:
                    self.g_back[s_n] = float("inf")
                if new_cost < self.g_back[s_n]:
                    self.g_back[s_n] = new_cost
                    self.PARENT_back[s_n] = s_back
                    self.OPEN_back.put(s_n, new_cost + self.h(s_n, self.s_start))

        return self.extract_path(s_meet), self.CLOSED_fore, self.CLOSED_back

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

    def extract_path(self, s_meet):
        """
        extract path from start and goal
        :param s_meet: meet point of bi-direction a*
        :return: path
        """

        # extract path for foreward part
        path_fore = [s_meet]
        s = s_meet

        while True:
            s = self.PARENT_fore[s]
            path_fore.append(s)
            if s == self.s_start:
                break

        # extract path for backward part
        path_back = []
        s = s_meet

        while True:
            s = self.PARENT_back[s]
            path_back.append(s)
            if s == self.s_goal:
                break

        return list(reversed(path_fore)) + list(path_back)

    def h(self, s, goal):
        """
        Calculate heuristic value.
        :param s: current node (state)
        :param goal: goal node (state)
        :return: heuristic value
        """

        heuristic_type = self.heuristic_type

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def cost(self, s_start, s_goal):
        """
        Calculate cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  cost for this motion
        :note: cost function could be more complicate!
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


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    bastar = BidirectionalAstar(x_start, x_goal, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)
    
    path, visited_fore, visited_back = bastar.searching()
    plot.animation_bi_astar(path, visited_fore, visited_back, "Bidirectional-A*")  # animation


if __name__ == '__main__':
    main()
