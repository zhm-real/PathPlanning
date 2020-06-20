#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

x_range, y_range = 14, 6     # size of background


def obs_map():
    """
    Initialize obstacles' positions

    :return: map of obstacles
    """

    obs = []
    for i in range(x_range):
        obs.append((i, 0))
    for i in range(x_range):
        obs.append((i, y_range - 1))

    for i in range(y_range):
        obs.append((0, i))
    for i in range(y_range):
        obs.append((x_range - 1, i))

    return obs


def lose_map():
    """
    Initialize losing states' positions
    :return: losing states
    """

    lose = []
    for i in range(2, 12):
        lose.append((i, 1))

    return lose


def get_reward(x_next, lose):
    """
    calculate reward of next state

    :param x_next: next state
    :return: reward
    """

    if x_next in lose:
        return -100                      # reward : -100, for lose states
    return -1                            # reward : -1, for other states


