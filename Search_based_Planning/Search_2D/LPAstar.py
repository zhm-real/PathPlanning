"""
LPA_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env


class LPAStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}

        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_start] = 0
        self.U[self.s_start] = self.CalculateKey(self.s_start)
        self.visited = set()
        self.count = 0

        self.fig = plt.figure()

    def run(self):
        self.Plot.plot_grid("Lifelong Planning A*")

        self.ComputeShortestPath()
        self.plot_path(self.extract_path())
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)

        plt.show()

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            self.visited = set()
            self.count += 1

            if (x, y) not in self.obs:
                self.obs.add((x, y))
            else:
                self.obs.remove((x, y))
                self.UpdateVertex((x, y))

            self.Plot.update_obs(self.obs)

            for s_n in self.get_neighbor((x, y)):
                self.UpdateVertex(s_n)

            self.ComputeShortestPath()

            plt.cla()
            self.Plot.plot_grid("Lifelong Planning A*")
            self.plot_visited(self.visited)
            self.plot_path(self.extract_path())
            self.fig.canvas.draw_idle()

    def ComputeShortestPath(self):
        while True:
            s, v = self.TopKey()

            if v >= self.CalculateKey(self.s_goal) and \
                    self.rhs[self.s_goal] == self.g[self.s_goal]:
                break

            self.U.pop(s)
            self.visited.add(s)

            if self.g[s] > self.rhs[s]:

                # Condition: over-consistent (eg: deleted obstacles)
                # So, rhs[s] decreased -- > rhs[s] < g[s]
                self.g[s] = self.rhs[s]
            else:

                # Condition: # under-consistent (eg: added obstacles)
                # So, rhs[s] increased --> rhs[s] > g[s]
                self.g[s] = float("inf")
                self.UpdateVertex(s)

            for s_n in self.get_neighbor(s):
                self.UpdateVertex(s_n)

    def UpdateVertex(self, s):
        """
        update the status and the current cost to come of state s.
        :param s: state s
        """

        if s != self.s_start:

            # Condition: cost of parent of s changed
            # Since we do not record the children of a state, we need to enumerate its neighbors
            self.rhs[s] = min(self.g[s_n] + self.cost(s_n, s)
                              for s_n in self.get_neighbor(s))

        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:

            # Condition: current cost to come is different to that of last time
            # state s should be added into OPEN set (set U)
            self.U[s] = self.CalculateKey(s)

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)

        return s, self.U[s]

    def CalculateKey(self, s):

        return [min(self.g[s], self.rhs[s]) + self.h(s),
                min(self.g[s], self.rhs[s])]

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        s_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                s_list.add(s_next)

        return s_list

    def h(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

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

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        for k in range(100):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_start:
                break

        return list(reversed(path))

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])


def main():
    x_start = (5, 5)
    x_goal = (45, 25)

    lpastar = LPAStar(x_start, x_goal, "Euclidean")
    lpastar.run()


if __name__ == '__main__':
    main()
