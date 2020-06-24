"""
RRT_2D
@author: huiming zhou
"""


from rrt_2D import env
from rrt_2D import plotting

import numpy as np
import math


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, x_start, x_goal, expand_len, goal_sample_rate, iter_limit):
        self.xI = Node(x_start)
        self.xG = Node(x_goal)
        self.expand_len = expand_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_limit = iter_limit
        self.vertex = [self.xI]

        self.env = env.Env()
        self.plotting = plotting.Plotting(x_start, x_goal)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_limit):
            node_rand = self.random_state(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.check_collision(node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.xG)

                if dist <= self.expand_len:
                    self.new_state(node_new, self.xG)
                    return self.extract_path(node_new)

        return None

    def random_state(self, goal_sample_rate):
        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.xG

    def nearest_neighbor(self, node_list, n):
        return self.vertex[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                          for nd in node_list]))]

    def new_state(self, node_start, node_end):
        node_new = Node((node_start.x, node_start.y))
        dist, theta = self.get_distance_and_angle(node_new, node_end)

        dist = min(self.expand_len, dist)
        node_new.x += dist * math.cos(theta)
        node_new.y += dist * math.sin(theta)
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.xG.x, self.xG.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def check_collision(self, node_end):
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


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 28)  # Goal node

    rrt = Rrt(x_start, x_goal, 0.4, 0.05, 2000)
    path = rrt.planning()

    if path:
        rrt.plotting.animation(rrt.vertex, path)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
