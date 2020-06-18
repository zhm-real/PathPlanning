#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import queue
import environment
import tools

class DFS:
    """
    DFS -> Depth-first Searching
    """

    def __init__(self, Start_State, Goal_State, n, m):
        self.xI = Start_State
        self.xG = Goal_State
        self.u_set = environment.motions                              # feasible input set
        self.obs_map = environment.map_obs()                          # position of obstacles
        self.n = n
        self.m = m

    def searching(self):
        """
        Searching using DFS.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_dfs = queue.QueueLIFO()                               # last-in-first-out queue
        q_dfs.put(self.xI)
        parent = {self.xI: self.xI}                             # record parents of nodes
        actions = {self.xI: (0, 0)}                             # record actions of nodes
        visited = []

        while not q_dfs.empty():
            x_current = q_dfs.get()
            visited.append(x_current)                           # record visited nodes
            if x_current == self.xG:                            # stop condition
                break
            for u_next in self.u_set:                           # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])   # neighbor node
                # if neighbor node is not in obstacles and has not been visited -> ...
                if 0 <= x_next[0] < self.n and 0 <= x_next[1] < self.m \
                        and x_next not in parent \
                        and not tools.obs_detect(x_current, u_next, self.obs_map):
                    q_dfs.put(x_next)
                    parent[x_next] = x_current
                    actions[x_next] = u_next
        [path_dfs, actions_dfs] = tools.extract_path(self.xI, self.xG, parent, actions)
        return path_dfs, actions_dfs, visited


if __name__ == '__main__':
    x_Start = (15, 10)       # Starting node
    x_Goal = (48, 15)        # Goal node
    dfs = DFS(x_Start, x_Goal, environment.col, environment.row)
    [path_dfs, actions_dfs, visited_dfs] = dfs.searching()
    tools.showPath(x_Start, x_Goal, path_dfs, visited_dfs, 'depth_first_searching')    # Plot path and visited nodes