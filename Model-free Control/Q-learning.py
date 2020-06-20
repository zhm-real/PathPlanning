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


class QLEARNING:
    def __init__(self, x_start, x_goal):
        self.u_set = motion_model.motions                       # feasible input set
        self.xI, self.xG = x_start, x_goal
        self.M = 500                                            # iteration numbers
        self.gamma = 0.9                                        # discount factor
        self.alpha = 0.5
        self.epsilon = 0.1                                      # epsilon error
        self.obs = env.obs_map()                                # position of obstacles
        self.lose = env.lose_map()                              # position of lose states
        self.name1 = "Qlearning, M=" + str(self.M)


    def Monte_Carlo(self):
        """
        Monte_Carlo experiments

        :return: Q_table, policy
        """

        Q_table = self.table_init()                                                 # Q_table initialization
        policy = {}                                                                 # policy table

        for k in range(self.M):                                                     # iterations
            x = self.state_init()                                                   # initial state
            while x != self.xG:                                                     # stop condition
                u = self.epsilon_greedy(int(np.argmax(Q_table[x])), self.epsilon)   # epsilon_greedy policy
                x_next = self.move_next(x, self.u_set[u])                           # next state
                reward = env.get_reward(x_next, self.lose)                          # reward observed
                Q_table[x][u] = (1 - self.alpha) * Q_table[x][u] + \
                                self.alpha * (reward + self.gamma * max(Q_table[x_next]))
                x = x_next

        for x in Q_table:
            policy[x] = int(np.argmax(Q_table[x]))                                  # extract policy

        return Q_table, policy


    def table_init(self):
        """
        Initialize Q_table: Q(s, a)
        :return: Q_table
        """

        Q_table = {}

        for i in range(env.x_range):
            for j in range(env.y_range):
                u = []
                if (i, j) not in self.obs:
                    for k in range(len(self.u_set)):
                        if (i, j) == self.xG:
                            u.append(0)
                        else:
                            u.append(np.random.random_sample())
                    Q_table[(i, j)] = u

        return Q_table


    def state_init(self):
        """
        initialize a starting state
        :return: starting state
        """
        while True:
            i = np.random.randint(0, env.x_range - 1)
            j = np.random.randint(0, env.y_range - 1)
            if (i, j) not in self.obs:
                return (i, j)


    def epsilon_greedy(self, u, error):
        """
        generate a policy using epsilon_greedy algorithm

        :param u: original input
        :param error: epsilon value
        :return: epsilon policy
        """

        if np.random.random_sample() < 3 / 4 * error:
            u_e = u
            while u_e == u:
                p = np.random.random_sample()
                if p < 0.25: u_e = 0
                elif p < 0.5: u_e = 1
                elif p < 0.75: u_e = 2
                else: u_e = 3
            return u_e
        return u


    def move_next(self, x, u):
        """
        get next state.

        :param x: current state
        :param u: input
        :return: next state
        """

        x_next = (x[0] + u[0], x[1] + u[1])
        if x_next in self.obs:
            return x
        return x_next


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
                if x_next == xG:
                    break
                else:
                    tools.plot_dots(x)                                  # each state in optimal path
                    path.append(x)
        plt.show()
        self.message()

        return path


    def message(self):
        print("starting state: ", self.xI)
        print("goal state: ", self.xG)
        print("iteration numbers: ", self.M)
        print("discount factor: ", self.gamma)
        print("epsilon error: ", self.epsilon)
        print("alpha: ", self.alpha)


if __name__ == '__main__':
    x_Start = (1, 1)
    x_Goal = (12, 1)

    Q_CALL = QLEARNING(x_Start, x_Goal)
    [value_SARSA, policy_SARSA] = Q_CALL.Monte_Carlo()
    path_VI = Q_CALL.simulation(x_Start, x_Goal, policy_SARSA)
