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


def plot_dots(x, length):
    plt.plot(x[0], x[1], linewidth='3', color='#808080', marker='o')
    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])
    if length % 15 == 0:
        plt.pause(0.001)

