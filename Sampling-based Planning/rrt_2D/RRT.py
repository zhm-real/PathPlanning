import env
import plotting

import numpy as np
import math


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RRT:
    def __init__(self, xI, xG):
        self.xI = Node(xI)
        self.xG = Node(xG)
        self.expand_len = 0.4
        self.goal_sample_rate = 0.05
        self.iterations = 5000
        self.node_list = [self.xI]

        self.env = env.Env()
        self.plotting = plotting.Plotting(xI, xG)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.path = self.planning()
        self.plotting.animation(self.node_list, self.path)

    def planning(self):
        for i in range(self.iterations):
            node_rand = self.random_state()
            node_near = self.nearest_neighbor(self.node_list, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if not self.check_collision(node_new):
                self.node_list.append(node_new)

            if self.dis_to_goal(self.node_list[-1]) <= self.expand_len:
                self.new_state(self.node_list[-1], self.xG)
                return self.extract_path(self.node_list)

        return None

    def random_state(self):
        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.xG

    def nearest_neighbor(self, node_list, n):
        return self.node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                             for nd in node_list]))]

    def new_state(self, node_start, node_end):
        node_new = Node((node_start.x, node_start.y))
        dist, theta = self.get_distance_and_angle(node_new, node_end)

        dist = min(self.expand_len, dist)
        node_new.x += dist * math.cos(theta)
        node_new.y += dist * math.sin(theta)
        node_new.parent = node_start

        return node_new

    def extract_path(self, nodelist):
        path = [(self.xG.x, self.xG.y)]
        node_now = nodelist[-1]

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def dis_to_goal(self, node_cal):
        return math.hypot(node_cal.x - self.xG.x, node_cal.y - self.xG.y)

    def check_collision(self, node_end):
        if node_end is None:
            return True

        for (ox, oy, r) in self.obs_circle:
            if math.hypot(node_end.x - ox, node_end.y - oy) <= r:
                return True

        for (ox, oy, w, h) in self.obs_rectangle:
            if 0 <= (node_end.x - ox) <= w and 0 <= (node_end.y - oy) <= h:
                return True

        for (ox, oy, w, h) in self.obs_boundary:
            if 0 <= (node_end.x - ox) <= w and 0 <= (node_end.y - oy) <= h:
                return True

        return False

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


if __name__ == '__main__':
    x_Start = (2, 2)  # Starting node
    x_Goal = (49, 28)  # Goal node

    rrt = RRT(x_Start, x_Goal)
