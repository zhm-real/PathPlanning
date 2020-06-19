#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import env
import tools
import motion_model

import matplotlib.pyplot as plt
import numpy as np
import sys

class Value_iteration:
    def __init__(self, x_start, x_goal):
        self.u_set = motion_model.motions                      # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001
        self.gamma = 0.9
        self.obs = env.obs_map()                               # position of obstacles
        self.lose = env.lose_map()
        self.name1 = "value_iteration, e=" + str(self.e) + ", gamma=" + str(self.gamma)
        self.name2 = "convergence of error"


    def iteration(self):
        value_table = {}
        policy = {}
        diff = []
        delta = sys.maxsize

        for i in range(env.x_range):
            for j in range(env.y_range):
                if (i, j) not in self.obs:
                    value_table[(i, j)] = 0

        while delta > self.e:
            x_value = 0
            for x in value_table:
                if x in self.xG: continue
                else:
                    value_list = []
                    for u in self.u_set:
                        [x_next, p_next] = motion_model.move_prob(x, u, self.obs)
                        value_list.append(self.cal_Q_value(x_next, p_next, value_table))
                    policy[x] = self.u_set[int(np.argmax(value_list))]
                    v_diff = abs(value_table[x] - max(value_list))
                    value_table[x] = max(value_list)
                    if v_diff > 0:
                        x_value = max(x_value, v_diff)
            delta = x_value
            diff.append(delta)
        return value_table, policy, diff


    def simulation(self, xI, xG, policy):
        path = []
        x = xI
        while x not in xG:
            u = policy[x]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next not in self.obs:
                x = x_next
                path.append(x)
        path.pop()
        return path


    def animation(self, path, diff):
        plt.figure(1)
        tools.show_map(self.xI, self.xG, self.obs, self.lose, self.name1)
        for x in path:
            tools.plot_dots(x)
        plt.show()

        plt.figure(2)
        plt.plot(diff, color='#808080', marker='o')
        plt.title(self.name2, fontdict=None)
        plt.xlabel('iterations')
        plt.grid('on')
        plt.show()


    def cal_Q_value(self, x, p, table):
        value = 0
        reward = self.get_reward(x)
        for i in range(len(x)):
            value += p[i] * (reward[i] + self.gamma * table[x[i]])
        return value


    def get_reward(self, x_next):
        reward = []
        for x in x_next:
            if x in self.xG:
                reward.append(10)
            elif x in self.lose:
                reward.append(-10)
            else:
                reward.append(0)
        return reward


if __name__ == '__main__':
    x_Start = (5, 5)
    x_Goal = [(49, 5), (49, 25)]

    VI = Value_iteration(x_Start, x_Goal)
    [value_VI, policy_VI, diff_VI] = VI.iteration()
    path_VI = VI.simulation(x_Start, x_Goal, policy_VI)

    VI.animation(path_VI, diff_VI)
