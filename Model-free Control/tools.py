#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt


def extract_path(xI, xG, parent, actions):
    """
    Extract the path based on the relationship of nodes.

    :param xI: Starting node
    :param xG: Goal node
    :param parent: Relationship between nodes
    :param actions: Action needed for transfer between two nodes
    :return: The planning path
    """

    path_back = [xG]
    acts_back = [actions[xG]]
    x_current = xG
    while True:
        x_current = parent[x_current]
        path_back.append(x_current)
        acts_back.append(actions[x_current])
        if x_current == xI: break

    return list(reversed(path_back)), list(reversed(acts_back))


def showPath(xI, xG, path):
    """
    Plot the path.

    :param xI: Starting node
    :param xG: Goal node
    :param path: Planning path
    :return: A plot
    """

    path.remove(xI)
    path.remove(xG)
    path_x = [path[i][0] for i in range(len(path))]
    path_y = [path[i][1] for i in range(len(path))]
    plt.plot(path_x, path_y, linewidth='5', color='r', linestyle='-')
    plt.pause(0.001)
    plt.show()


def show_map(xI, xG, obs_map, lose_map, name):
    """
    Plot the background you designed.

    :param xI: starting state
    :param xG: goal states
    :param obs_map: positions of obstacles
    :param lose_map: positions of losing state
    :param name: name of this figure
    :return: a figure
    """

    obs_x = [obs_map[i][0] for i in range(len(obs_map))]
    obs_y = [obs_map[i][1] for i in range(len(obs_map))]

    lose_x = [lose_map[i][0] for i in range(len(lose_map))]
    lose_y = [lose_map[i][1] for i in range(len(lose_map))]

    plt.plot(xI[0], xI[1], "bs", ms = 24)                                    # plot starting state (blue)
    plt.plot(xG[0], xG[1], "gs", ms = 24)                                    # plot goal states (green)

    plt.plot(obs_x, obs_y, "sk", ms = 24)                                    # plot obstacles (black)
    plt.plot(lose_x, lose_y, marker = 's', color = '#808080', ms = 24)       # plot losing states (grown)
    plt.title(name, fontdict=None)
    plt.axis("equal")


def plot_dots(x):
    """
    Plot state x for animation

    :param x: current node
    :return: a plot
    """

    plt.plot(x[0], x[1], linewidth='3', color='#808080', marker='o', ms = 24)    # plot dots for animation
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    plt.pause(0.001)


