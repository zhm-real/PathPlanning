import env
import plotting
import node

import numpy as np
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None


class RRT:
    def __init__(self, xI, xG):
        self.xI = Node(xI[0], xI[1])
        self.xG = node.Node(xG[0], xG[1])

        self.env = env.Env()
        # self.plotting = plotting.Plotting(xI, xG)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range

        # self.obs_boundary = self.env.obs_boundary
        # self.obs_circle = self.env.obs_circle

        self.obstacleList = [
            (5, 5, 1),
            (3, 6, 2),
            (3, 8, 2),
            (3, 10, 2),
            (7, 5, 2),
            (9, 5, 2),
            (8, 10, 1)
        ]  # [x, y, radius]

        self.expand_range = 0.8
        self.goal_sample_rate = 0.05
        self.iterations = 500
        self.node_list = []

        path = self.planning()

        if path is None:
            print("No path!")
        else:
            print("get it!")

        self.draw_graph()

        plt.plot([x[0] for x in path], [x[1] for x in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()

    def planning(self):
        self.node_list = [self.xI]
        for i in range(self.iterations):
            node_rand = self.generate_random_node()
            node_near = self.get_nearest_node(self.node_list, node_rand)

            node_new = self.new_node(node_near, node_rand, self.expand_range)

            if not self.check_collision(node_new, self.obstacleList):
                self.node_list.append(node_new)

            self.draw_graph(node_rand)

            if self.cal_dis_to_goal(self.node_list[-1]) <= self.expand_range:
                node_end = self.new_node(self.node_list[-1], node.Node(self.xG.x, self.xG.y), self.expand_range)
                if not self.check_collision(node_end, self.obstacleList):
                    return self.extract_path(self.node_list)

        return None

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node_x in self.node_list:
            if node_x.parent:
                plt.plot(node_x.path_x, node_x.path_y, "-g")

        for (ox, oy, size) in self.obstacleList:
            self.plot_circle(ox, oy, size)

        plt.plot(self.xI.x, self.xI.y, "xr")
        plt.plot(self.xG.x, self.xG.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def extract_path(self, nodelist):
        path = [(self.xG.x, self.xG.y)]
        node_now = nodelist[-1]

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def cal_dis_to_goal(self, node_cal):
        return math.hypot((node_cal.x - self.xG.x), (node_cal.y - self.xG.y))

    def new_node(self, node_start, node_goal, expand_range):
        new_node = node.Node(node_start.x, node_start.y)
        d, theta = self.calc_distance_and_angle(new_node, node_goal)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if d < expand_range:
            expand_range = d

        new_node.x += expand_range * math.cos(theta)
        new_node.y += expand_range * math.sin(theta)
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)

        new_node.parent = node_start

        return new_node

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            return node.Node(np.random.uniform(self.x_range[0], self.x_range[1]),
                             np.random.uniform(self.y_range[0], self.y_range[1]))
        else:
            return node.Node(self.xG.x, self.xG.y)

    def get_nearest_node(self, node_list, node_random):
        dlist = [(nod.x - node_random.x) ** 2 + (nod.y - node_random.y) ** 2
                 for nod in node_list]
        minind = dlist.index(min(dlist))

        return self.node_list[minind]

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def check_collision(self, node_check, obstacleList):

        if node_check is None:
            return True

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node_check.path_x]
            dy_list = [oy - y for y in node_check.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return True  # collision

        return False  # safe
    #
    # def check_collision(self, node_check):
    #     for obs in self.obs_boundary:
    #         dx = node_check.x - obs[0]
    #         dy = node_check.y - obs[1]
    #         if 0 <= dx <= obs[2] and 0 <= dy <= obs[2]:
    #             return True
    #
    #     for obs in self.obs_circle:
    #         d = (node_check.x - obs[0]) ** 2 + (node_check.y - obs[1]) ** 2
    #         if d <= obs[2] ** 2:
    #             return True
    #
    #     return False


if __name__ == '__main__':
    x_Start = (0, 0)  # Starting node
    x_Goal = (6, 10)  # Goal node

    rrt = RRT(x_Start, x_Goal)
