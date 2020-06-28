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

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g_fore = {self.xI: 0, self.xG: float("inf")}
        self.g_back = {self.xG: 0, self.xI: float("inf")}

        self.OPEN_fore = queue.QueuePrior()
        self.OPEN_fore.put(self.xI, self.g_fore[self.xI] + self.h(self.xI, self.xG))
        self.OPEN_back = queue.QueuePrior()
        self.OPEN_back.put(self.xG, self.g_back[self.xG] + self.h(self.xG, self.xI))

        self.CLOSED_fore = []
        self.CLOSED_back = []

        self.Parent_fore = {self.xI: self.xI}
        self.Parent_back = {self.xG: self.xG}

    def searching(self):
        visited_fore, visited_back = [], []
        s_meet = self.xI

        while not self.OPEN_fore.empty() and not self.OPEN_back.empty():

            # solve foreward-search
            s_fore = self.OPEN_fore.get()
            if s_fore in self.Parent_back:
                s_meet = s_fore
                break
            visited_fore.append(s_fore)
            for u in self.u_set:
                s_next = tuple([s_fore[i] + u[i] for i in range(len(s_fore))])
                if s_next not in self.obs:
                    new_cost = self.g_fore[s_fore] + self.get_cost(s_fore, u)
                    if s_next not in self.g_fore:
                        self.g_fore[s_next] = float("inf")
                    if new_cost < self.g_fore[s_next]:
                        self.g_fore[s_next] = new_cost
                        self.Parent_fore[s_next] = s_fore
                        self.OPEN_fore.put(s_next, new_cost + self.h(s_next, self.xG))

            # solve backward-search
            s_back = self.OPEN_back.get()
            if s_back in self.Parent_fore:
                s_meet = s_back
                break
            visited_back.append(s_back)
            for u in self.u_set:
                s_next = tuple([s_back[i] + u[i] for i in range(len(s_back))])
                if s_next not in self.obs:
                    new_cost = self.g_back[s_back] + self.get_cost(s_back, u)
                    if s_next not in self.g_back:
                        self.g_back[s_next] = float("inf")
                    if new_cost < self.g_back[s_next]:
                        self.g_back[s_next] = new_cost
                        self.Parent_back[s_next] = s_back
                        self.OPEN_back.put(s_next, new_cost + self.h(s_next, self.xI))

        return self.extract_path(s_meet), visited_fore, visited_back

    def extract_path(self, s):
        path_back_fore = [s]
        s_current = s

        while True:
            s_current = self.Parent_fore[s_current]
            path_back_fore.append(s_current)

            if s_current == self.xI:
                break

        path_back_back = []
        s_current = s

        while True:
            s_current = self.Parent_back[s_current]
            path_back_back.append(s_current)

            if s_current == self.xG:
                break

        return list(reversed(path_back_fore)) + list(path_back_back)

    def h(self, state, goal):
        """
        Calculate heuristic.
        :param state: current node (state)
        :param goal: goal node (state)
        :return: heuristic
        """

        heuristic_type = self.heuristic_type

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
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
    x_start = (5, 5)  # Starting node
    x_goal = (49, 25)  # Goal node

    bastar = BidirectionalAstar(x_start, x_goal, "euclidean")
    plot = plotting.Plotting(x_start, x_goal)  # class Plotting

    fig_name = "Bidirectional-A* Algorithm"
    path, v_fore, v_back = bastar.searching()
    plot.animation_bi_astar(path, v_fore, v_back, fig_name)  # animation generate


if __name__ == '__main__':
    main()
