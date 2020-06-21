#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import env
import plotting
import motion_model

import numpy as np
import sys

class Q_value_iteration:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001                                          # threshold for convergence
        self.gamma = 0.9                                        # discount factor

        self.env = env.Env(self.xI, self.xG)                            # class Env
        self.motion = motion_model.Motion_model(self.xI, self.xG)       # class Motion_model
        self.plotting = plotting.Plotting(self.xI, self.xG)             # class Plotting

        self.u_set = self.env.motions                                   # feasible input set
        self.stateSpace = self.env.stateSpace                           # state space
        self.obs = self.env.obs_map()                                   # position of obstacles
        self.lose = self.env.lose_map()                                 # position of lose states

        self.name1 = "Q-value_iteration, gamma=" + str(self.gamma)
        self.name2 = "converge process, e=" + str(self.e)

        [self.value, self.policy, self.diff] = self.iteration(self.xI, self.xG)
        self.path = self.extract_path(self.xI, self.xG, self.policy)
        self.plotting.animation(self.path, self.name1)
        self.plotting.plot_diff(self.diff, self.name2)


    def iteration(self, xI, xG):
        """
        Q_value_iteration
        :return: converged Q table and policy
        """

        Q_table = {}
        policy = {}
        diff = []
        delta = sys.maxsize
        count = 0

        for x in self.stateSpace:
            Q_table[x] = [0, 0, 0, 0]                       # initialize Q_table

        while delta > self.e:                               # convergence condition
            count += 1
            x_value = 0
            for x in self.stateSpace:
                if x not in x_Goal:
                    for k in range(len(self.u_set)):
                        [x_next, p_next] = self.motion.move_next(x, self.u_set[k])
                        Q_value = self.cal_Q_value(x_next, p_next, Q_table)
                        v_diff = abs(Q_table[x][k] - Q_value)
                        Q_table[x][k] = Q_value
                        if v_diff > 0:
                            x_value = max(x_value, v_diff)
            diff.append(x_value)
            delta = x_value

        for x in self.stateSpace:
            if x not in xG:
                policy[x] = np.argmax(Q_table[x])

        self.message(count)

        return Q_table, policy, diff


    def cal_Q_value(self, x, p, table):
        """
        cal Q_value.

        :param x: next state vector
        :param p: probability of each state
        :param table: value table
        :return: Q-value
        """

        value = 0
        reward = self.env.get_reward(x)                  # get reward of next state
        for i in range(len(x)):
            value += p[i] * (reward[i] + self.gamma * max(table[x[i]]))

        return value


    def extract_path(self, xI, xG, policy):
        """
        extract path from converged policy.

        :param xI: starting state
        :param xG: goal states
        :param policy: converged policy
        :return: path
        """

        x, path = xI, [xI]
        while x not in xG:
            u = self.u_set[policy[x]]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next in self.obs:
                print("Collision! Please run again!")
                break
            else:
                path.append(x_next)
                x = x_next
        return path


    def message(self, count):
        """
        print important message.

        :param count: iteration numbers
        :return: print
        """

        print("starting state: ", self.xI)
        print("goal states: ", self.xG)
        print("condition for convergence: ", self.e)
        print("discount factor: ", self.gamma)
        print("iteration times: ", count)


if __name__ == '__main__':
    x_Start = (5, 5)
    x_Goal = [(49, 5), (49, 25)]

    QVI = Q_value_iteration(x_Start, x_Goal)
