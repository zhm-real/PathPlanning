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
        self.env = env.Env()
        self.obs = self.env.obs_map()


    def animation(self, path, visited, name):
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)


    def plot_grid(self, name):
        obs_x = [self.obs[i][0] for i in range(len(self.obs))]
        obs_y = [self.obs[i][1] for i in range(len(self.obs))]

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")


    def plot_visited(self, visited):
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
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]

        plt.plot(path_x, path_y, linewidth='3', color='r', marker='o')
        plt.pause(0.01)
        plt.show()
