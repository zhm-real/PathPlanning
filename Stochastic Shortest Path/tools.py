#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt
import environment


def obs_detect(x, u, obs_map):
    """
    Detect if the next state is in obstacles using this input.

    :param x: current state
    :param u: input
    :param obs_map: map of obstacles
    :return: in obstacles: True / not in obstacles: False
    """

    x_next = [x[0] + u[0], x[1] + u[1]]                   # next state using input 'u'
    if u not in environment.motions or \
            obs_map[x_next[0]][x_next[1]] == 1:           # if 'u' is feasible and next state is not in obstacles
        return True
    return False


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


def showPath(xI, xG, path, visited, name):
    """
    Plot the path.

    :param xI: Starting node
    :param xG: Goal node
    :param path: Planning path
    :param visited: Visited nodes
    :param name: Name of this figure
    :return: A plot
    """

    background = environment.obstacles()
    fig, ax = plt.subplots()
    for k in range(len(visited)):
        background[visited[k][1]][visited[k][0]] = [.5, .5, .5]    # visited nodes: gray color
    for k in range(len(path)):
        background[path[k][1]][path[k][0]] = [1., 0., 0.]          # path: red color
    background[xI[1]][xI[0]] = [0., 0., 1.]                        # starting node: blue color
    background[xG[1]][xG[0]] = [0., 1., .5]                        # goal node: green color
    ax.imshow(background)
    ax.invert_yaxis()                                              # put origin of coordinate to left-bottom
    plt.title(name, fontdict=None)
    plt.show()

