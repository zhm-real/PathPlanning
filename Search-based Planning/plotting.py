#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt


def animation(xI, xG, obs, path, visited, name):
    """
    generate animation for exploring process of algorithm

    :param xI: starting state
    :param xG: goal state
    :param obs: obstacle map
    :param path: optimal path
    :param visited: visited nodes
    :param name: name of this figure
    :return: animation
    """

    visited.remove(xI)
    path.remove(xI)
    path.remove(xG)

    # plot gridworld
    obs_x = [obs[i][0] for i in range(len(obs))]
    obs_y = [obs[i][1] for i in range(len(obs))]
    plt.plot(xI[0], xI[1], "bs")
    plt.plot(xG[0], xG[1], "gs")
    plt.plot(obs_x, obs_y, "sk")
    plt.title(name)
    plt.axis("equal")

    # animation for the exploring order of visited nodes
    count = 0
    for x in visited:
        count += 1
        plt.plot(x[0], x[1], linewidth='3', color='#808080', marker='o')
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if count < 500: length = 20
        elif count < 700: length = 30
        else: length = 50

        if count % length == 0: plt.pause(0.001)

    # plot optimal path
    path_x = [path[i][0] for i in range(len(path))]
    path_y = [path[i][1] for i in range(len(path))]
    plt.plot(path_x, path_y, linewidth='3', color='r', marker='o')

    # show animation
    plt.pause(0.01)
    plt.show()
