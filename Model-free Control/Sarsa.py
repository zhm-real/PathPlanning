#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import env
import plotting
import motion_model

import numpy as np


class SARSA:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.M = 500  # iteration numbers
        self.gamma = 0.9  # discount factor
        self.alpha = 0.5
        self.epsilon = 0.1

        self.env = env.Env(self.xI, self.xG)
        self.motion = motion_model.Motion_model(self.xI, self.xG)
        self.plotting = plotting.Plotting(self.xI, self.xG)

        self.u_set = self.env.motions  # feasible input set
        self.stateSpace = self.env.stateSpace  # state space
        self.obs = self.env.obs_map()  # position of obstacles
        self.lose = self.env.lose_map()  # position of lose states

        self.name1 = "SARSA, M=" + str(self.M)

        [self.value, self.policy] = self.Monte_Carlo(self.xI, self.xG)
        self.path = self.extract_path(self.xI, self.xG, self.policy)
        self.plotting.animation(self.path, self.name1)


    def Monte_Carlo(self, xI, xG):
        """
        Monte_Carlo experiments

        :return: Q_table, policy
        """

        Q_table = self.table_init()                                             # Q_table initialization
        policy = {}                                                             # policy table

        for k in range(self.M):                                                 # iterations
            x = self.state_init()                                               # initial state
            u = self.epsilon_greedy(int(np.argmax(Q_table[x])), self.epsilon)
            while x != xG:                                                 # stop condition
                x_next = self.move_next(x, self.u_set[u])                       # next state
                reward = self.env.get_reward(x_next)                      # reward observed
                u_next = self.epsilon_greedy(int(np.argmax(Q_table[x_next])), self.epsilon)
                Q_table[x][u] = (1 - self.alpha) * Q_table[x][u] + \
                                self.alpha * (reward + self.gamma * Q_table[x_next][u_next])
                x, u = x_next, u_next

        for x in Q_table:
            policy[x] = int(np.argmax(Q_table[x]))                              # extract policy

        return Q_table, policy


    def table_init(self):
        """
        Initialize Q_table: Q(s, a)
        :return: Q_table
        """

        Q_table = {}

        for x in self.stateSpace:
            u = []
            if x not in self.obs:
                for k in range(len(self.u_set)):
                    if x == self.xG:
                        u.append(0)
                    else:
                        u.append(np.random.random_sample())
                    Q_table[x] = u
        return Q_table


    def state_init(self):
        """
        initialize a starting state
        :return: starting state
        """
        while True:
            i = np.random.randint(0, self.env.x_range - 1)
            j = np.random.randint(0, self.env.y_range - 1)
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

    def message(self):
        """
        print important message.

        :param count: iteration numbers
        :return: print
        """

        print("starting state: ", self.xI)
        print("goal state: ", self.xG)
        print("iteration numbers: ", self.M)
        print("discount factor: ", self.gamma)
        print("epsilon error: ", self.epsilon)
        print("alpha: ", self.alpha)


if __name__ == '__main__':
    x_Start = (1, 1)
    x_Goal = (12, 1)

    SARSA_CALL = SARSA(x_Start, x_Goal)
