#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import tools
import env
import motion_model

class Astar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.u_set = motion_model.motions                   # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.obs = env.obs_map()                            # position of obstacles
        self.heuristic_type = heuristic_type

        env.show_map(self.xI, self.xG, self.obs, "a_star searching")

    def searching(self):
        """
        Searching using A_star.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_astar = queue.QueuePrior()                            # priority queue
        q_astar.put(self.xI, 0)
        parent = {self.xI: self.xI}                             # record parents of nodes
        action = {self.xI: (0, 0)}                              # record actions of nodes
        cost = {self.xI: 0}

        while not q_astar.empty():
            x_current = q_astar.get()
            if x_current == self.xG:                            # stop condition
                break
            if x_current != self.xI:
                tools.plot_dots(x_current, len(parent))
            for u_next in self.u_set:                           # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                if x_next not in self.obs:
                    new_cost = cost[x_current] + self.get_cost(x_current, u_next)
                    if x_next not in cost or new_cost < cost[x_next]:           # conditions for updating cost
                        cost[x_next] = new_cost
                        priority = new_cost + self.Heuristic(x_next, self.xG, self.heuristic_type)
                        q_astar.put(x_next, priority)           # put node into queue using priority "f+h"
                        parent[x_next] = x_current
                        action[x_next] = u_next
        [path_astar, actions_astar] = tools.extract_path(self.xI, self.xG, parent, action)
        return path_astar, actions_astar

    def get_cost(self, x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1

    def Heuristic(self, state, goal, heuristic_type):
        """
        Calculate heuristic.

        :param state: current node (state)
        :param goal: goal node (state)
        :param heuristic_type: choosing different heuristic functions
        :return: heuristic
        """

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


if __name__ == '__main__':
    x_Start = (5, 5)                # Starting node
    x_Goal = (49, 5)                # Goal node
    astar = Astar(x_Start, x_Goal, "manhattan")
    [path_astar, actions_astar] = astar.searching()
    tools.showPath(x_Start, x_Goal, path_astar)      # Plot path and visited nodes