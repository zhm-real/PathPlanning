#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""
import numpy as np

motions = [(1, 0), (-1, 0), (0, 1), (0, -1)]                # feasible motion sets


def move_prob(x, u, obs, eta = 0.2):
    """
    Motion model of robots,

    :param x: current state (node)
    :param u: input
    :param obs: obstacle map
    :param eta: noise in motion model
    :return: next states and corresponding probability
    """

    p_next = [1 - eta, eta / 2, eta / 2]
    x_next = []
    if u == (0, 1):
        u_real = [(0, 1), (-1, 0), (1, 0)]
    elif u == (0, -1):
        u_real = [(0, -1), (-1, 0), (1, 0)]
    elif u == (-1, 0):
        u_real = [(-1, 0), (0, 1), (0, -1)]
    else:
        u_real = [(1, 0), (0, 1), (0, -1)]

    for act in u_real:
        if (x[0] + act[0], x[1] + act[1]) in obs:
            x_next.append(x)
        else:
            x_next.append((x[0] + act[0], x[1] + act[1]))

    return x_next, p_next