"""
RTAAstar 2D (Real-time Adaptive A*)
@author: huiming zhou
"""

import os
import sys
import copy
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import queue, plotting, env


class RTAAStar:
    def __init__(self, s_start, s_goal, N, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.N = N  # number of expand nodes each iteration
        self.visited = []  # order of visited nodes in planning
        self.path = []  # path of each iteration
        self.h_table = {}  # h_value table

    def init(self):
        """
        initialize the h_value of all nodes in the environment.
        it is a global table.
        """

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.h_table[(i, j)] = self.h((i, j))

    def searching(self):
        self.init()
        s_start = self.s_start  # initialize start node

        while True:
            OPEN, CLOSED, g_table, PARENT = \
                self.Astar(s_start, self.N)

            if OPEN == "FOUND":  # reach the goal node
                self.path.append(CLOSED)
                break

            s_next, h_value = self.cal_h_value(OPEN, CLOSED, g_table, PARENT)

            for x in h_value:
                self.h_table[x] = h_value[x]

            s_start, path_k = self.extract_path_in_CLOSE(s_start, s_next, h_value)
            self.path.append(path_k)

    def cal_h_value(self, OPEN, CLOSED, g_table, PARENT):
        v_open = {}
        h_value = {}
        for (_, x) in OPEN.enumerate():
            v_open[x] = g_table[PARENT[x]] + 1 + self.h_table[x]
        s_open = min(v_open, key=v_open.get)
        f_min = v_open[s_open]
        for x in CLOSED:
            h_value[x] = f_min - g_table[x]

        return s_open, h_value

    def iteration(self, CLOSED):
        h_value = {}

        for s in CLOSED:
            h_value[s] = float("inf")  # initialize h_value of CLOSED nodes

        while True:
            h_value_rec = copy.deepcopy(h_value)
            for s in CLOSED:
                h_list = []
                for s_n in self.get_neighbor(s):
                    if s_n not in CLOSED:
                        h_list.append(self.cost(s, s_n) + self.h_table[s_n])
                    else:
                        h_list.append(self.cost(s, s_n) + h_value[s_n])
                h_value[s] = min(h_list)  # update h_value of current node

            if h_value == h_value_rec:  # h_value table converged
                return h_value

    def Astar(self, x_start, N):
        OPEN = queue.QueuePrior()  # OPEN set
        OPEN.put(x_start, self.h_table[x_start])
        CLOSED = []  # CLOSED set
        g_table = {x_start: 0, self.s_goal: float("inf")}  # Cost to come
        PARENT = {x_start: x_start}  # relations
        count = 0  # counter

        while not OPEN.empty():
            count += 1
            s = OPEN.get()
            CLOSED.append(s)

            if s == self.s_goal:  # reach the goal node
                self.visited.append(CLOSED)
                return "FOUND", self.extract_path(x_start, PARENT), [], []

            for s_n in self.get_neighbor(s):
                if s_n not in CLOSED:
                    new_cost = g_table[s] + self.cost(s, s_n)
                    if s_n not in g_table:
                        g_table[s_n] = float("inf")
                    if new_cost < g_table[s_n]:  # conditions for updating Cost
                        g_table[s_n] = new_cost
                        PARENT[s_n] = s
                        OPEN.put(s_n, g_table[s_n] + self.h_table[s_n])

            if count == N:  # expand needed CLOSED nodes
                break

        self.visited.append(CLOSED)  # visited nodes in each iteration

        return OPEN, CLOSED, g_table, PARENT

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

    def extract_path_in_CLOSE(self, s_end, s_start, h_value):
        path = [s_start]
        s = s_start

        while True:
            h_list = {}
            for s_n in self.get_neighbor(s):
                if s_n in h_value:
                    h_list[s_n] = h_value[s_n]
            s_key = max(h_list, key=h_list.get)  # move to the smallest node with min h_value
            path.append(s_key)  # generate path
            s = s_key  # use end of this iteration as the start of next

            if s_key == s_end:  # reach the expected node in OPEN set
                return s_start, list(reversed(path))

    def extract_path(self, x_start, parent):
        """
        Extract the path based on the relationship of nodes.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = parent[s]
            path.append(s)
            if s == x_start:
                break

        return list(reversed(path))

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

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


def main():
    s_start = (10, 5)
    s_goal = (45, 25)

    rtaa = RTAAStar(s_start, s_goal, 240, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)

    rtaa.searching()
    plot.animation_lrta(rtaa.path, rtaa.visited,
                        "Real-time Adaptive A* (RTAA*)")


if __name__ == '__main__':
    main()
