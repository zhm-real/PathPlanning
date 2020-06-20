#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import plotting
import env


class BFS:
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
        Searching using BFS.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_bfs = queue.QueueFIFO()                                     # first-in-first-out queue
        q_bfs.put(xI)
        parent = {xI: xI}                                   # record parents of nodes
        action = {xI: (0, 0)}                                    # record actions of nodes
        visited = []

        while not q_bfs.empty():
            x_current = q_bfs.get()
            if x_current == xG:
                break
            visited.append(x_current)
            for u_next in self.u_set:                                 # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                if x_next not in parent and x_next not in self.obs:   # node not visited and not in obstacles
                    q_bfs.put(x_next)
                    parent[x_next], action[x_next] = x_current, u_next

        [path, policy] = self.extract_path(xI, xG, parent, action)     # extract path

        return path, policy, visited


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
    x_Start = (5, 5)                    # Starting node
    x_Goal = (49, 5)                    # Goal node
    bfs = BFS(x_Start, x_Goal)
