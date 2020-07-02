"""
D_star 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import plotting
from Search_2D import env


class Dstar:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()
        self.Plot = plotting.Plotting(self.xI, self.xG)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.fig = plt.figure()
        self.OPEN = set()
        self.t = {}
        self.PARENT = {}
        self.h = {self.xG: 0}
        self.k = {}
        self.path = []

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0
                self.PARENT[(i, j)] = None

    def run(self, s_start, s_end):
        self.insert(s_end, 0)
        while True:
            self.process_state()
            if self.t[s_start] == 'CLOSED':
                break
        self.path = self.extract_path(s_start, s_end)
        self.Plot.plot_grid("Dynamic A* (D*)")
        self.plot_path(self.path)
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Add obstacle at: x =", x, ",", "y =", y)
            self.obs.add((x, y))
            plt.plot(x, y, 'sk')
            if (x, y) in self.path:
                s = self.xI
                while s != self.xG:
                    if self.PARENT[s] in self.obs:
                        self.modify(s)
                        continue
                    s = self.PARENT[s]
                self.path = self.extract_path(self.xI, self.xG)
                self.plot_path(self.path)
            self.fig.canvas.draw_idle()

    def extract_path(self, s_start, s_end):
        path = []
        s = s_start
        while True:
            s = self.PARENT[s]
            if s == s_end:
                return path
            path.append(s)

    def process_state(self):
        s = self.min_state()
        if s is None:
            return -1
        k_old = self.get_k_min()
        self.delete(s)

        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.h[s_n] <= k_old and self.h[s] > self.h[s_n] + self.cost(s_n, s):
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)
        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                        (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n):
                        self.insert(s, self.h[s])
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:
                            self.insert(s_n, self.h[s_n])
        return self.get_k_min()

    def min_state(self):
        if not self.OPEN:
            return None
        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        if not self.OPEN:
            return -1
        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)
        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'
        self.OPEN.remove(s)

    def modify(self, s):
        self.modify_cost(s)
        while True:
            k_min = self.process_state()
            if k_min >= self.h[s]:
                break

    def modify_cost(self, s):
        if self.t[s] == 'CLOSED':
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def cost(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return float("inf")
        return 1

    @staticmethod
    def plot_path(path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, marker='o')


def main():
    s_start = (5, 5)
    s_goal = (45, 25)
    dstar = Dstar(s_start, s_goal)
    dstar.run(s_start, s_goal)


if __name__ == '__main__':
    main()
