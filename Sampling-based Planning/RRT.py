import env
import plotting

import numpy as np
import math


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.path_x = []
        self.path_y = []
        self.parent = None


class RRT:
    def __init__(self, xI, xG):
        self.xI = Node(xI)
        self.xG = Node(xG)
        self.expand_len = 0.8
        self.goal_sample_rate = 0.05
        self.iterations = 5000
        self.node_list = [self.xI]

        self.env = env.Env()
        self.plotting = plotting.Plotting(xI, xG)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs
        self.obs_rectangle = self.env.obs_boundary

        self.path = self.planning()
        self.plotting.animation(self.node_list, self.path)

    def planning(self):
        for i in range(self.iterations):
            node_rand = self.generate_random_node()
            node_near = self.get_nearest_node(self.node_list, node_rand)
            node_new = self.new_node(node_near, node_rand, self.expand_len)

            if not self.check_collision(node_new, self.obs_circle, self.obs_rectangle):
                self.node_list.append(node_new)

            if self.cal_dis_to_goal(self.node_list[-1]) <= self.expand_len:
                self.new_node(self.node_list[-1], self.xG, self.expand_len)
                return self.extract_path(self.node_list)

        return None

    def extract_path(self, nodelist):
        path = [(self.xG.x, self.xG.y)]
        node_now = nodelist[-1]

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def cal_dis_to_goal(self, node_cal):
        return math.hypot(node_cal.x - self.xG.x, node_cal.y - self.xG.y)

    def new_node(self, node_start, node_goal, expand_len):
        new_node = Node((node_start.x, node_start.y))
        d, theta = self.calc_distance_and_angle(new_node, node_goal)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if d < expand_len:
            expand_len = d

        new_node.x += expand_len * math.cos(theta)
        new_node.y += expand_len * math.sin(theta)
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)

        new_node.parent = node_start

        return new_node

    def generate_random_node(self):
        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.xG

    def get_nearest_node(self, node_list, n):
        return self.node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                             for nd in node_list]))]

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def check_collision(node_end, obs_circle, obs_rectangle):
        if node_end is None:
            return True

        for (ox, oy, r) in obs_circle:
            if math.hypot(node_end.x - ox, node_end.y - oy) <= r:
                return True

        for (ox, oy, w, h) in obs_rectangle:
            if 0 <= (node_end.x - ox) <= w and 0 <= (node_end.y - oy) <= h:
                return True

        return False


if __name__ == '__main__':
    x_Start = (15, 5)  # Starting node
    x_Goal = (45, 25)  # Goal node

    rrt = RRT(x_Start, x_Goal)
