#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import env
import tools
import motion_model

class Dijkstra:
    def __init__(self, x_start, x_goal, x_range, y_range):
        self.u_set = motion_model.motions                      # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.x_range, self.y_range = x_range, y_range
        self.obs = env.obs_map()                               # position of obstacles

        env.show_map(self.xI, self.xG, self.obs, "dijkstra searching")

    def searching(self):
        """
        Searching using Dijkstra.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_dijk = queue.QueuePrior()                            # priority queue
        q_dijk.put(self.xI, 0)
        parent = {self.xI: self.xI}                            # record parents of nodes
        action = {self.xI: (0, 0)}                             # record actions of nodes
        cost = {self.xI: 0}

        while not q_dijk.empty():
            x_current = q_dijk.get()
            if x_current == self.xG:                           # stop condition
                break
            if x_current != self.xI:
                tools.plot_dots(x_current, len(parent))
            for u_next in self.u_set:                          # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                if x_next not in self.obs:                     # node not visited and not in obstacles
                    new_cost = cost[x_current] + self.get_cost(x_current, u_next)
                    if x_next not in cost or new_cost < cost[x_next]:
                        cost[x_next] = new_cost
                        priority = new_cost
                        q_dijk.put(x_next, priority)           # put node into queue using cost to come as priority
                        parent[x_next] = x_current
                        action[x_next] = u_next
        [path_dijk, action_dijk] = tools.extract_path(self.xI, self.xG, parent, action)
        return path_dijk, action_dijk

    def get_cost(self, x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1


if __name__ == '__main__':
    x_Start = (5, 5)                # Starting node
    x_Goal = (49, 5)                # Goal node
    dijkstra = Dijkstra(x_Start, x_Goal, env.x_range, env.y_range)
    [path_dijk, actions_dijk] = dijkstra.searching()
    tools.showPath(x_Start, x_Goal, path_dijk)