#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

class Env():
    def __init__(self, xI, xG):
        self.x_range = 14  # size of background
        self.y_range = 6
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

        return obs

    def lose_map(self):
        """
        Initialize losing states' positions
        :return: losing states
        """

        lose = []
        for i in range(2, 12):
            lose.append((i, 1))

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

        if x_next in self.lose:
            return -100  # reward : -100, for lose states
        return -1  # reward : -1, for other states
