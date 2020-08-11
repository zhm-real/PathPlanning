"""
Anytime_D_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting
from Search_2D import env


class ADStar:
    def __init__(self, s_start, s_goal, eps, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(s_start, s_goal)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.OPEN = {}, {}, {}

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.eps = eps
        self.OPEN[self.s_goal] = self.Key(self.s_goal)
        self.CLOSED, self.INCONS = set(), dict()

        self.visited = set()
        self.count = 0
        self.count_env_change = 0
        self.obs_add = set()
        self.obs_remove = set()
        self.title = "Anytime D*: Small changes"  # Significant changes
        self.fig = plt.figure()

    def run(self):
        self.Plot.plot_grid(self.title)
        self.ComputeOrImprovePath()
        self.plot_visited()
        self.plot_path(self.extract_path())
        self.visited = set()

        while True:
            if self.eps <= 1.0:
                break
            self.eps -= 0.5
            self.OPEN.update(self.INCONS)
            for s in self.OPEN:
                self.OPEN[s] = self.Key(s)
            self.CLOSED = set()
            self.ComputeOrImprovePath()
            self.plot_visited()
            self.plot_path(self.extract_path())
            self.visited = set()
            plt.pause(0.5)

        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            self.count_env_change += 1
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            # for small changes
            if self.title == "Anytime D*: Small changes":
                if (x, y) not in self.obs:
                    self.obs.add((x, y))
                    self.g[(x, y)] = float("inf")
                    self.rhs[(x, y)] = float("inf")
                else:
                    self.obs.remove((x, y))
                    self.UpdateState((x, y))

                self.Plot.update_obs(self.obs)

                for sn in self.get_neighbor((x, y)):
                    self.UpdateState(sn)

                plt.cla()
                self.Plot.plot_grid(self.title)

                while True:
                    if len(self.INCONS) == 0:
                        break
                    self.OPEN.update(self.INCONS)
                    for s in self.OPEN:
                        self.OPEN[s] = self.Key(s)
                    self.CLOSED = set()
                    self.ComputeOrImprovePath()
                    self.plot_visited()
                    self.plot_path(self.extract_path())
                    # plt.plot(self.title)
                    self.visited = set()

                    if self.eps <= 1.0:
                        break

            else:
                if (x, y) not in self.obs:
                    self.obs.add((x, y))
                    self.obs_add.add((x, y))
                    plt.plot(x, y, 'sk')
                    if (x, y) in self.obs_remove:
                        self.obs_remove.remove((x, y))
                else:
                    self.obs.remove((x, y))
                    self.obs_remove.add((x, y))
                    plt.plot(x, y, marker='s', color='white')
                    if (x, y) in self.obs_add:
                        self.obs_add.remove((x, y))

                self.Plot.update_obs(self.obs)

                if self.count_env_change >= 15:
                    self.count_env_change = 0
                    self.eps += 2.0
                    for s in self.obs_add:
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")

                        for sn in self.get_neighbor(s):
                            self.UpdateState(sn)

                    for s in self.obs_remove:
                        for sn in self.get_neighbor(s):
                            self.UpdateState(sn)
                        self.UpdateState(s)

                    plt.cla()
                    self.Plot.plot_grid(self.title)

                    while True:
                        if self.eps <= 1.0:
                            break
                        self.eps -= 0.5
                        self.OPEN.update(self.INCONS)
                        for s in self.OPEN:
                            self.OPEN[s] = self.Key(s)
                        self.CLOSED = set()
                        self.ComputeOrImprovePath()
                        self.plot_visited()
                        self.plot_path(self.extract_path())
                        plt.title(self.title)
                        self.visited = set()
                        plt.pause(0.5)

            self.fig.canvas.draw_idle()

    def ComputeOrImprovePath(self):
        while True:
            s, v = self.TopKey()
            if v >= self.Key(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break

            self.OPEN.pop(s)
            self.visited.add(s)

            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                self.CLOSED.add(s)
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
            else:
                self.g[s] = float("inf")
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
                self.UpdateState(s)

    def UpdateState(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:
            if s not in self.CLOSED:
                self.OPEN[s] = self.Key(s)
            else:
                self.INCONS[s] = 0

    def Key(self, s):
        if self.g[s] > self.rhs[s]:
            return [self.rhs[s] + self.eps * self.h(self.s_start, s), self.rhs[s]]
        else:
            return [self.g[s] + self.h(self.s_start, s), self.g[s]]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.OPEN, key=self.OPEN.get)
        return s, self.OPEN[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(100):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break

        return list(path)

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def plot_visited(self):
        self.count += 1

        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in self.visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    dstar = ADStar(s_start, s_goal, 2.5, "euclidean")
    dstar.run()


if __name__ == '__main__':
    main()
