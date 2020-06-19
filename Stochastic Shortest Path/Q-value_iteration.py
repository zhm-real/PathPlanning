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


class Q_value_iteration:
    def __init__(self, x_start, x_goal):
        self.u_set = motion_model.motions                           # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001                                              # threshold for convergence
        self.gamma = 0.9                                            # discount factor
        self.obs = env.obs_map()                                    # position of obstacles
        self.lose = env.lose_map()                                  # position of lose states
        self.name1 = "Q-value_iteration, e=" + str(self.e) \
                     + ", gamma=" + str(self.gamma)
        self.name2 = "convergence of error"


    def iteration(self):
        """
        Q_value_iteration
        :return: converged Q table and policy
        """

        Q_table = {}
        policy = {}
        delta = sys.maxsize
        count = 0

        for i in range(env.x_range):
            for j in range(env.y_range):
                if (i, j) not in self.obs:
                    Q_table[(i, j)] = [0, 0, 0, 0]          # initialize Q_table

        while delta > self.e:                               # convergence condition
            count += 1
            x_value = 0
            for x in Q_table:
                if x not in x_Goal:
                    for k in range(len(self.u_set)):
                        [x_next, p_next] = motion_model.move_prob(x, self.u_set[k], self.obs)
                        Q_value = self.cal_Q_value(x_next, p_next, Q_table)
                        v_diff = abs(Q_table[x][k] - Q_value)
                        Q_table[x][k] = Q_value
                        if v_diff > 0:
                            x_value = max(x_value, v_diff)
            delta = x_value

        for x in Q_table:
            if x not in x_Goal:
                policy[x] = np.argmax(Q_table[x])

        self.message(count)

        return Q_table, policy


    def cal_Q_value(self, x, p, table):
        """
        cal Q_value.

        :param x: next state vector
        :param p: probability of each state
        :param table: value table
        :return: Q-value
        """

        value = 0
        reward = env.get_reward(x, self.xG, self.lose)                  # get reward of next state
        for i in range(len(x)):
            value += p[i] * (reward[i] + self.gamma * max(table[x[i]]))

        return value


    def simulation(self, xI, xG, policy):
        """
        simulate a path using converged policy.

        :param xI: starting state
        :param xG: goal state
        :param policy: converged policy
        :return: simulation path
        """

        plt.figure(1)                                                   # path animation
        tools.show_map(xI, xG, self.obs, self.lose, self.name1)         # show background

        x, path = xI, []
        while True:
            u = self.u_set[policy[x]]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next in self.obs:
                print("Collision!")                                     # collision: simulation failed
            else:
                x = x_next
                if x_next in xG:
                    break
                else:
                    tools.plot_dots(x)                                  # each state in optimal path
                    path.append(x)
        plt.show()

        return path


    def message(self, count):
        print("starting state: ", self.xI)
        print("goal states: ", self.xG)
        print("condition for convergence: ", self.e)
        print("discount factor: ", self.gamma)
        print("iteration times: ", count)


if __name__ == '__main__':
    x_Start = (5, 5)
    x_Goal = [(49, 5), (49, 25)]

    QVI = Q_value_iteration(x_Start, x_Goal)
    [value_QVI, policy_QVI] = QVI.iteration()
    path_VI = QVI.simulation(x_Start, x_Goal, policy_QVI)
