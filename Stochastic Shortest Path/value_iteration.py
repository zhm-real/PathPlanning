#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import env
import tools
import motion_model

import numpy as np
import copy

class Value_iteration:
    def __init__(self, x_start, x_goal):
        self.u_set = motion_model.motions                      # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.T = 500
        self.gamma = 0.9
        self.obs = env.obs_map()                               # position of obstacles
        self.lose = env.lose_map()
        self.name = "value_iteration, T=" + str(self.T) + ", gamma=" + str(self.gamma)

        env.show_map(self.xI, self.xG, self.obs, self.lose, self.name)

    def iteration(self):
        value_table = {}
        policy = {}

        for i in range(env.x_range):
            for j in range(env.y_range):
                if (i, j) not in self.obs:
                    value_table[(i, j)] = 0

        for k in range(self.T):
            value_table_update = copy.deepcopy(value_table)
            for key in value_table:
                


if __name__ == '__main__':
    x_Start = (5, 5)                # Starting node
    x_Goal = (49, 5)                # Goal node
    VI = Value_iteration(x_Start, x_Goal)