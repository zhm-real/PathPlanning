#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import env
import plotting


class Dijkstra:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()
        self.u_set = self.Env.motions                               # feasible input set
        self.obs = self.Env.obs                                     # position of obstacles
        [self.path, self.policy, self.visited] = self.searching(self.xI, self.xG)

        self.fig_name = "Dijkstra's Algorithm"
        plotting.animation(self.xI, self.xG, self.obs,
                           self.path, self.visited, self.fig_name)  # animation generate


    def searching(self, xI, xG):
        """
        Searching using Dijkstra.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_dijk = queue.QueuePrior()                             # priority queue
        q_dijk.put(xI, 0)
        parent = {xI: xI}                                       # record parents of nodes
        action = {xI: (0, 0)}                                   # record actions of nodes
        visited = []                                            # record visited nodes
        cost = {xI: 0}

        while not q_dijk.empty():
            x_current = q_dijk.get()
            if x_current == xG:                                 # stop condition
                break
            visited.append(x_current)
            for u_next in self.u_set:                           # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                if x_next not in self.obs:                      # node not visited and not in obstacles
                    new_cost = cost[x_current] + self.get_cost(x_current, u_next)
                    if x_next not in cost or new_cost < cost[x_next]:
                        cost[x_next] = new_cost
                        priority = new_cost
                        q_dijk.put(x_next, priority)            # put node into queue using cost to come as priority
                        parent[x_next], action[x_next] = x_current, u_next

        [path, policy] = self.extract_path(xI, xG, parent, action)

        return path, policy, visited


    def get_cost(self, x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1


    def extract_path(self, xI, xG, parent, policy):
        """
        Extract the path based on the relationship of nodes.

        :param xI: Starting node
        :param xG: Goal node
        :param parent: Relationship between nodes
        :param policy: Action needed for transfer between two nodes
        :return: The planning path
        """

        path_back = [xG]
        acts_back = [policy[xG]]
        x_current = xG
        while True:
            x_current = parent[x_current]
            path_back.append(x_current)
            acts_back.append(policy[x_current])
            if x_current == xI: break

        return list(path_back), list(acts_back)


if __name__ == '__main__':
    x_Start = (5, 5)                # Starting node
    x_Goal = (49, 5)                # Goal node
    dijkstra = Dijkstra(x_Start, x_Goal)
