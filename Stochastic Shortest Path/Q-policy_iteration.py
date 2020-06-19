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
import copy
import sys


class Q_policy_iteration:
    def __init__(self, x_start, x_goal):
        self.u_set = motion_model.motions  # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001
        self.gamma = 0.9
        self.obs = env.obs_map()  # position of obstacles
        self.lose = env.lose_map()
        self.name1 = "policy_iteration, e=" + str(self.e) + ", gamma=" + str(self.gamma)
        self.name2 = "convergence of error"


    def policy_evaluation(self, policy, value):
        delta = sys.maxsize

        while delta > self.e:
            x_value = 0
            for x in value:
                if x not in self.xG:
                    for k in range(len(self.u_set)):
                        [x_next, p_next] = motion_model.move_prob(x, self.u_set[k], self.obs)
                        v_Q = self.cal_Q_value(x_next, p_next, policy, value)
                        v_diff = abs(value[x][k] - v_Q)
                        value[x][k] = v_Q
                        if v_diff > 0:
                            x_value = max(x_value, v_diff)
            delta = x_value
        return value


    def policy_improvement(self, policy, value):
        for x in value:
            if x not in self.xG:
                policy[x] = int(np.argmax(value[x]))
        return policy


    def iteration(self):
        Q_table = {}
        policy = {}

        for i in range(env.x_range):
            for j in range(env.y_range):
                if (i, j) not in self.obs:
                    Q_table[(i, j)] = [0, 0, 0, 0]
                    policy[(i, j)] = 0

        while True:
            policy_back = copy.deepcopy(policy)
            Q_table = self.policy_evaluation(policy, Q_table)
            policy = self.policy_improvement(policy, Q_table)
            if policy_back == policy: break
        return Q_table, policy


    def simulation(self, xI, xG, policy):
        path = []
        x = xI
        while x not in xG:
            u = self.u_set[policy[x]]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next not in self.obs:
                x = x_next
                path.append(x)
        path.pop()
        return path


    def animation(self, path):
        plt.figure(1)
        tools.show_map(self.xI, self.xG, self.obs, self.lose, self.name1)
        for x in path:
            tools.plot_dots(x)
        plt.show()


    def cal_Q_value(self, x, p, policy, table):
        value = 0
        reward = self.get_reward(x)
        for i in range(len(x)):
            value += p[i] * (reward[i] + self.gamma * table[x[i]][policy[x[i]]])
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

    QPI = Q_policy_iteration(x_Start, x_Goal)
    [value_QPI, policy_QPI] = QPI.iteration()
    path_QPI = QPI.simulation(x_Start, x_Goal, policy_QPI)

    QPI.animation(path_QPI)
