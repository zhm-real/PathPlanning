#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: huiming zhou
"""

import matplotlib.pyplot as plt
import env

class Plotting():
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.env = env.Env(self.xI, self.xG)
        self.obs = self.env.obs_map()
        self.lose = self.env.lose_map()


    def animation(self, path, name):
        """
        animation.

        :param path: optimal path
        :param name: tile of figure
        :return: an animation
        """

        plt.figure(1)
        self.plot_grid(name)
        self.plot_lose()
        self.plot_path(path)


    def plot_grid(self, name):
        """
        plot the obstacles in environment.

        :param name: title of figure
        :return: plot
        """

        obs_x = [self.obs[i][0] for i in range(len(self.obs))]
        obs_y = [self.obs[i][1] for i in range(len(self.obs))]

        plt.plot(self.xI[0], self.xI[1], "bs", ms = 24)
        plt.plot(self.xG[0], self.xG[1], "gs", ms = 24)

        plt.plot(obs_x, obs_y, "sk", ms = 24)
        plt.title(name)
        plt.axis("equal")


    def plot_lose(self):
        """
        plot losing states in environment.
        :return: a plot
        """

        lose_x = [self.lose[i][0] for i in range(len(self.lose))]
        lose_y = [self.lose[i][1] for i in range(len(self.lose))]

        plt.plot(lose_x, lose_y, color = '#A52A2A', marker = 's', ms = 24)


    def plot_visited(self, visited):
        """
        animation of order of visited nodes.

        :param visited: visited nodes
        :return: animation
        """

        visited.remove(self.xI)
        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], linewidth='3', color='#808080', marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event:
            [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 15
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 45

            if count % length == 0: plt.pause(0.001)


    def plot_path(self, path):
        path.remove(self.xI)
        path.remove(self.xG)

        for x in path:
            plt.plot(x[0], x[1], color='#808080', marker='o', ms = 23)
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event:
            [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)
        plt.show()
        plt.pause(0.5)


    def plot_diff(self, diff, name):
        plt.figure(2)
        plt.title(name, fontdict=None)
        plt.xlabel('iterations')
        plt.ylabel('difference of successive iterations')
        plt.grid('on')

        count = 0
        for x in diff:
            plt.plot(count, x, color='#808080', marker='o')  # plot dots for animation
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event:
            [exit(0) if event.key == 'escape' else None])
            plt.pause(0.07)
            count += 1

        plt.plot(diff, color='#808080')
        plt.pause(0.01)
        plt.show()
