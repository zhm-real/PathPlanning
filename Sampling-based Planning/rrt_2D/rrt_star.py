"""
RRT_star 2D
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
        self.cost = 0.0
        self.parent = None


class RrtStar:
    def __init__(self, x_start, x_goal, expand_len,
                 goal_sample_rate, search_radius, iter_limit):
        self.xI = Node(x_start)
        self.xG = Node(x_goal)
        self.expand_len = expand_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
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
        for k in range(self.iter_limit):
            node_rand = self.random_state(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.check_collision(node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                if neighbor_index:
                    node_new = self.choose_parent(node_new, neighbor_index)
                    self.vertex.append(node_new)
                    self.rewire(node_new, neighbor_index)

        index = self.search_goal_parent()
        return self.extract_path(self.vertex[index])

    def random_state(self, goal_sample_rate):
        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.xG

    def nearest_neighbor(self, node_list, n):
        return self.vertex[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                          for nd in node_list]))]

    def new_state(self, node_start, node_goal):
        node_new = Node((node_start.x, node_start.y))
        dist, theta = self.get_distance_and_angle(node_new, node_goal)
        dist = min(self.expand_len, dist)

        node_new.x += dist * math.cos(theta)
        node_new.y += dist * math.sin(theta)
        node_new.parent = node_start

        return node_new

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.expand_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]

        return [dist_table.index(d) for d in dist_table if d <= r]

    def choose_parent(self, node_new, neighbor_index):
        cost = []

        for i in neighbor_index:
            node_neighbor = self.vertex[i]
            cost.append(self.get_new_cost(node_neighbor, node_new))

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new = self.new_state(self.vertex[cost_min_index], node_new)
        node_new.cost = min(cost)

        return node_new

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.xG.x, n.y - self.xG.y) for n in self.vertex]
        node_index = [dist_list.index(i) for i in dist_list if i <= self.expand_len]

        if node_index:
            cost_list = [dist_list[i] + self.vertex[i].cost for i in node_index]
            return node_index[int(np.argmin(cost_list))]

        return None

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]
            new_cost = self.get_new_cost(node_new, node_neighbor)

            if node_neighbor.cost > new_cost:
                self.vertex[i] = self.new_state(node_new, node_neighbor)
                self.propagate_cost_to_leaves(node_new)

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)
        return node_start.cost + dist

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.vertex:
            if node.parent == parent_node:
                node.cost = self.get_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def extract_path(self, node_end):
        path = [[self.xG.x, self.xG.y]]
        node = node_end
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

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

    rrt_star = RrtStar(x_start, x_goal, 1, 0.1, 10, 5000)
    path = rrt_star.planning()

    if path:
        rrt_star.plotting.animation(rrt_star.vertex, path)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
