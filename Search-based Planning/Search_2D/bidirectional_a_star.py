"""
Bidirectional_a_star 2D
@author: huiming zhou
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class BidirectionalAstar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()                                    # class Env

        self.u_set = self.Env.motions                           # feasible input set
        self.obs = self.Env.obs                                 # position of obstacles

        self.g_fore = {self.xI: 0, self.xG: float("inf")}       # cost to come: from x_start
        self.g_back = {self.xG: 0, self.xI: float("inf")}       # cost to come: form x_goal

        self.OPEN_fore = queue.QueuePrior()                     # OPEN set for foreward searching
        self.OPEN_fore.put(self.xI, self.g_fore[self.xI] + self.h(self.xI, self.xG))
        self.OPEN_back = queue.QueuePrior()                     # OPEN set for backward searching
        self.OPEN_back.put(self.xG, self.g_back[self.xG] + self.h(self.xG, self.xI))

        self.CLOSED_fore = []                                   # CLOSED set for foreward
        self.CLOSED_back = []                                   # CLOSED set for backward

        self.PARENT_fore = {self.xI: self.xI}
        self.PARENT_back = {self.xG: self.xG}

    def searching(self):
        s_meet = self.xI

        while not self.OPEN_fore.empty() and not self.OPEN_back.empty():
            # solve foreward-search
            s_fore = self.OPEN_fore.get()
            if s_fore in self.PARENT_back:
                s_meet = s_fore
                break
            self.CLOSED_fore.append(s_fore)

            for u in self.u_set:
                s_next = tuple([s_fore[i] + u[i] for i in range(2)])
                if s_next not in self.obs:
                    new_cost = self.g_fore[s_fore] + self.get_cost(s_fore, u)
                    if s_next not in self.g_fore:
                        self.g_fore[s_next] = float("inf")
                    if new_cost < self.g_fore[s_next]:
                        self.g_fore[s_next] = new_cost
                        self.PARENT_fore[s_next] = s_fore
                        self.OPEN_fore.put(s_next, new_cost + self.h(s_next, self.xG))

            # solve backward-search
            s_back = self.OPEN_back.get()
            if s_back in self.PARENT_fore:
                s_meet = s_back
                break
            self.CLOSED_back.append(s_back)

            for u in self.u_set:
                s_next = tuple([s_back[i] + u[i] for i in range(len(s_back))])
                if s_next not in self.obs:
                    new_cost = self.g_back[s_back] + self.get_cost(s_back, u)
                    if s_next not in self.g_back:
                        self.g_back[s_next] = float("inf")
                    if new_cost < self.g_back[s_next]:
                        self.g_back[s_next] = new_cost
                        self.PARENT_back[s_next] = s_back
                        self.OPEN_back.put(s_next, new_cost + self.h(s_next, self.xI))

        return self.extract_path(s_meet), self.CLOSED_fore, self.CLOSED_back

    def extract_path(self, s_meet):
        # extract path for foreward part
        path_fore = [s_meet]
        s = s_meet

        while True:
            s = self.PARENT_fore[s]
            path_fore.append(s)
            if s == self.xI:
                break

        # extract path for backward part
        path_back = []
        s = s_meet

        while True:
            s = self.PARENT_back[s]
            path_back.append(s)
            if s == self.xG:
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
        elif heuristic_type == "euclidean":
            return ((goal[0] - s[0]) ** 2 + (goal[1] - s[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")

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

    bastar = BidirectionalAstar(x_start, x_goal, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)
    fig_name = "Bidirectional-A*"
    
    path, visited_fore, visited_back = bastar.searching()
    plot.animation_bi_astar(path, visited_fore, visited_back, fig_name)  # animation


if __name__ == '__main__':
    main()
