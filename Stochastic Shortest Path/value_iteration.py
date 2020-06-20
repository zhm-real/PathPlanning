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
        self.e = 0.001                                         # threshold for convergence
        self.gamma = 0.9                                       # discount factor
        self.obs = env.obs_map()                               # position of obstacles
        self.lose = env.lose_map()                             # position of lose states
        self.name1 = "value_iteration, gamma=" + str(self.gamma)
        self.name2 = "converge process, e=" + str(self.e)


    def iteration(self):
        """
        value_iteration.

        :return: converged value table, optimal policy and variation of difference,
        """

        value_table = {}                        # value table
        policy = {}                             # policy
        diff = []                               # maximum difference between two successive iteration
        delta = sys.maxsize                     # initialize maximum difference
        count = 0                               # iteration times

        for i in range(env.x_range):
            for j in range(env.y_range):
                if (i, j) not in self.obs:
                    value_table[(i, j)] = 0     # initialize value table for feasible states

        while delta > self.e:                   # converged condition
            count += 1
            x_value = 0
            for x in value_table:
                if x not in self.xG:
                    value_list = []
                    for u in self.u_set:
                        [x_next, p_next] = motion_model.move_prob(x, u, self.obs)           # recall motion model
                        value_list.append(self.cal_Q_value(x_next, p_next, value_table))    # cal Q value
                    policy[x] = self.u_set[int(np.argmax(value_list))]                      # update policy
                    v_diff = abs(value_table[x] - max(value_list))                          # maximum difference
                    value_table[x] = max(value_list)                                        # update value table
                    if v_diff > 0:
                        x_value = max(x_value, v_diff)
            delta = x_value                                                                 # update delta
            diff.append(delta)
        self.message(count)                                                                 # print key parameters

        return value_table, policy, diff


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
            value += p[i] * (reward[i] + self.gamma * table[x[i]])      # cal Q-value

        return value


    def simulation(self, xI, xG, policy, diff):
        """
        simulate a path using converged policy.

        :param xI: starting state
        :param xG: goal state
        :param policy: converged policy
        :return: simulation path
        """

        plt.figure(1)                                               # path animation
        tools.show_map(xI, xG, self.obs, self.lose, self.name1)     # show background

        x, path = xI, []
        while True:
            u = policy[x]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next in self.obs:
                print("Collision!")                                 # collision: simulation failed
            else:
                x = x_next
                if x_next in xG: break
                else:
                    tools.plot_dots(x)                              # each state in optimal path
                    path.append(x)
        plt.pause(1)

        plt.figure(2)                                               # difference between two successive iteration
        plt.plot(diff, color='#808080', marker='o')
        plt.title(self.name2, fontdict=None)
        plt.xlabel('iterations')
        plt.ylabel('difference of successive iterations')
        plt.grid('on')
        plt.show()

        return path


    def message(self, count):
        print("starting state: ", self.xI)
        print("goal states: ", self.xG)
        print("condition for convergence: ", self.e)
        print("discount factor: ", self.gamma)
        print("iteration times: ", count)


if __name__ == '__main__':
    x_Start = (5, 5)                    # starting state
    x_Goal = [(49, 5), (49, 25)]        # goal states

    VI = Value_iteration(x_Start, x_Goal)
    [value_VI, policy_VI, diff_VI] = VI.iteration()
    path_VI = VI.simulation(x_Start, x_Goal, policy_VI, diff_VI)

