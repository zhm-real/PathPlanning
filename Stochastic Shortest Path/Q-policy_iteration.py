#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import env
import plotting
import motion_model

import numpy as np
import copy
import sys

class Q_policy_iteration:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001                                  # threshold for convergence
        self.gamma = 0.9                                # discount factor

        self.env = env.Env(self.xI, self.xG)                        # class Env
        self.motion = motion_model.Motion_model(self.xI, self.xG)   # class Motion_model
        self.plotting = plotting.Plotting(self.xI, self.xG)         # class Plotting

        self.u_set = self.env.motions                               # feasible input set
        self.stateSpace = self.env.stateSpace                       # state space
        self.obs = self.env.obs_map()                               # position of obstacles
        self.lose = self.env.lose_map()                             # position of lose states

        self.name1 = "Q-policy_iteration, gamma=" + str(self.gamma)

        [self.value, self.policy] = self.iteration()
        self.path = self.extract_path(self.xI, self.xG, self.policy)
        self.plotting.animation(self.path, self.name1)


    def policy_evaluation(self, policy, value):
        """
        evaluation process using current policy.

        :param policy: current policy
        :param value: value table
        :return: converged value table
        """

        delta = sys.maxsize

        while delta > self.e:               # convergence condition
            x_value = 0
            for x in value:
                if x not in self.xG:
                    for k in range(len(self.u_set)):
                        [x_next, p_next] = self.motion.move_next(x, self.u_set[k])
                        v_Q = self.cal_Q_value(x_next, p_next, policy, value)
                        v_diff = abs(value[x][k] - v_Q)
                        value[x][k] = v_Q
                        if v_diff > 0:
                            x_value = max(x_value, v_diff)
            delta = x_value

        return value


    def policy_improvement(self, policy, value):
        """
        policy improvement process.

        :param policy: policy table
        :param value: current value table
        :return: improved policy
        """

        for x in self.stateSpace:
            if x not in self.xG:
                policy[x] = int(np.argmax(value[x]))

        return policy


    def iteration(self):
        """
        Q-policy iteration
        :return: converged policy and its value table.
        """

        Q_table = {}
        policy = {}
        count = 0

        for x in self.stateSpace:
            Q_table[x] = [0, 0, 0, 0]              # initialize Q_value table
            policy[x] = 0                          # initialize policy table

        while True:
            count += 1
            policy_back = copy.deepcopy(policy)
            Q_table = self.policy_evaluation(policy, Q_table)   # evaluation process
            policy = self.policy_improvement(policy, Q_table)   # improvement process
            if policy_back == policy: break                     # convergence condition

        self.message(count)

        return Q_table, policy


    def cal_Q_value(self, x, p, policy, table):
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
            value += p[i] * (reward[i] + self.gamma * table[x[i]][policy[x[i]]])

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

    QPI = Q_policy_iteration(x_Start, x_Goal)
