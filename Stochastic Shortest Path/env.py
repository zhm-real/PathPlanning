#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

class Env():
    def __init__(self, xI, xG):
        self.x_range = 51           # size of background
        self.y_range = 31
        self.motions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        self.xI = xI
        self.xG = xG
        self.obs = self.obs_map()
        self.lose = self.lose_map()
        self.stateSpace = self.state_space()


    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = []

        for i in range(x):
            obs.append((i, 0))
        for i in range(x):
            obs.append((i, y - 1))

        for i in range(y):
            obs.append((0, i))
        for i in range(y):
            obs.append((x - 1, i))

        for i in range(10, 21):
            obs.append((i, 15))
        for i in range(15):
            obs.append((20, i))

        for i in range(15, 30):
            obs.append((30, i))
        for i in range(16):
            obs.append((40, i))

        return obs


    def lose_map(self):
        """
        Initialize losing states' positions
        :return: losing states
        """

        lose = []
        for i in range(25, 36):
            lose.append((i, 13))

        return lose


    def state_space(self):
        """
        generate state space
        :return: state space
        """

        state_space = []
        for i in range(self.x_range):
            for j in range(self.y_range):
                if (i, j) not in self.obs:
                    state_space.append((i, j))

        return state_space


    def get_reward(self, x_next):
        """
        calculate reward of next state

        :param x_next: next state
        :return: reward
        """

        reward = []
        for x in x_next:
            if x in self.xG:
                reward.append(10)  # reward : 10, for goal states
            elif x in self.lose:
                reward.append(-10)  # reward : -10, for lose states
            else:
                reward.append(0)  # reward : 0, for other states

        return reward
