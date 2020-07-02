"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

from Search_2D import queue
from Search_2D import plotting
from Search_2D import env


class DStar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(x_start, x_goal)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.U = queue.QueuePrior()  # priority queue / U set
        self.g, self.rhs = {}, {}
        self.km = 0

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.xG] = 0
        self.U.put(self.xG, self.Key(self.xG))
        self.fig = plt.figure()

    def run(self):
        self.Plot.plot_grid("Dynamic A* (D*)")
        self.ComputePath()
        self.plot_path(self.extract_path())
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: x =", x, ",", "y =", y)

            s_curr = self.xI
            s_last = self.xI
            i = 0
            path = []

            while s_curr != self.xG:
                s_list = {}
                for s in self.get_neighbor(s_curr):
                    s_list[s] = self.g[s] + self.get_cost(s_curr, s)
                s_curr = min(s_list, key=s_list.get)
                path.append(s_curr)

                if i < 1:
                    self.km += self.h(s_last, s_curr)
                    s_last = s_curr
                    if (x, y) not in self.obs:
                        self.obs.add((x, y))
                        plt.plot(x, y, 'sk')
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")
                    else:
                        self.obs.remove((x, y))
                        plt.plot(x, y, marker='s', color='white')
                        self.UpdateVertex((x, y))
                    for s in self.get_neighbor((x, y)):
                        self.UpdateVertex(s)
                    i += 1
                self.ComputePath()
                self.plot_path(path)
                self.fig.canvas.draw_idle()

    @staticmethod
    def plot_path(path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, marker='o')

    def ComputePath(self):
        while self.U.top_key() < self.Key(self.xI) or \
                self.rhs[self.xI] != self.g[self.xI]:
            k_old = self.U.top_key()
            s = self.U.get()
            if k_old < self.Key(s):
                self.U.put(s, self.Key(s))
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)

    def UpdateVertex(self, s):
        if s != self.xG:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.get_cost(s, x))
        self.U.remove(s)
        if self.g[s] != self.rhs[s]:
            self.U.put(s, self.Key(s))

    def Key(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.xI, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type            # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def get_cost(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return float("inf")
        return 1

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        path = []
        s = self.xI
        count = 0
        while True:
            count += 1
            g_list = {}
            for x in self.get_neighbor(s):
                g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            if s == self.xG or count > 100:
                return list(reversed(path))
            path.append(s)

    def print_g(self):
        print("he")
        for k in range(self.Env.y_range):
            j = self.Env.y_range - k - 1
            string = ""
            for i in range(self.Env.x_range):
                if self.g[(i, j)] == float("inf"):
                    string += ("00" + ', ')
                else:
                    if self.g[(i, j)] // 10 == 0:
                        string += ("0" + str(self.g[(i, j)]) + ', ')
                    else:
                        string += (str(self.g[(i, j)]) + ', ')
            print(string)


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    dstar = DStar(x_start, x_goal, "euclidean")
    dstar.run()


if __name__ == '__main__':
    main()
