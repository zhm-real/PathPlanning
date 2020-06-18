#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import environment
import tools

class Dijkstra:
    def __init__(self, Start_State, Goal_State, n, m):
        self.xI = Start_State
        self.xG = Goal_State
        self.u_set = environment.motions                       # feasible input set
        self.obs_map = environment.map_obs()                   # position of obstacles
        self.n = n
        self.m = m

    def searching(self):
        """
        Searching using Dijkstra.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_dijk = queue.QueuePrior()                            # priority queue
        q_dijk.put(self.xI, 0)
        parent = {self.xI: self.xI}                            # record parents of nodes
        actions = {self.xI: (0, 0)}                            # record actions of nodes
        cost = {self.xI: 0}
        visited = []

        while not q_dijk.empty():
            x_current = q_dijk.get()
            visited.append(x_current)                          # record visited nodes
            if x_current == self.xG:                           # stop condition
                break
            for u_next in self.u_set:                          # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                # if neighbor node is not in obstacles -> ...
                if 0 <= x_next[0] < self.n and 0 <= x_next[1] < self.m \
                        and not tools.obs_detect(x_current, u_next, self.obs_map):
                    new_cost = cost[x_current] + int(self.get_cost(x_current, u_next))
                    if x_next not in cost or new_cost < cost[x_next]:
                        cost[x_next] = new_cost
                        priority = new_cost
                        q_dijk.put(x_next, priority)           # put node into queue using cost to come as priority
                        parent[x_next] = x_current
                        actions[x_next] = u_next
        [path_dijk, actions_dijk] = tools.extract_path(self.xI, self.xG, parent, actions)
        return path_dijk, actions_dijk, visited

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
    x_Start = (15, 10)              # Starting node
    x_Goal = (48, 15)               # Goal node
    dijkstra = Dijkstra(x_Start, x_Goal, environment.col, environment.row)
    [path_dijk, actions_dijk, visited_dijk] = dijkstra.searching()
    tools.showPath(x_Start, x_Goal, path_dijk, visited_dijk, 'dijkstra_searching')