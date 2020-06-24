import env
import plotting

import numpy as np
import math


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.cost = 0.0
        self.parent = None


class RRT:
    def __init__(self, xI, xG):
        self.xI = Node(xI)
        self.xG = Node(xG)
        self.expand_len = 1
        self.goal_sample_rate = 0.05
        self.connect_dist = 10
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
        self.plotting.animation(self.node_list, self.path, False)

    def planning(self):
        for k in range(self.iterations):
            node_rand = self.random_state()
            node_near = self.nearest_neighbor(self.node_list, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if not self.check_collision(node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                node_new = self.choose_parent(node_new, neighbor_index)
                if node_new:
                    self.node_list.append(node_new)
                    self.rewire(node_new, neighbor_index)

            # if self.dis_to_goal(self.node_list[-1]) <= self.expand_len:
            #     self.new_state(self.node_list[-1], self.xG)
            #     return self.extract_path()

        index = self.search_best_goal_node()
        self.xG.parent = self.node_list[index]
        return self.extract_path()

    def random_state(self):
        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0], self.x_range[1]),
                         np.random.uniform(self.y_range[0], self.y_range[1])))
        return self.xG

    def nearest_neighbor(self, node_list, n):
        return self.node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
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
        n = len(self.node_list) + 1
        r = min(self.connect_dist * math.sqrt((math.log(n) / n)), self.expand_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.node_list]
        node_index = [dist_table.index(d) for d in dist_table if d <= r]

        return node_index

    def choose_parent(self, node_new, neighbor_index):
        if not neighbor_index:
            return None

        cost = []

        for i in neighbor_index:
            node_near = self.node_list[i]
            node_mid = self.new_state(node_near, node_new)

            if node_mid and not self.check_collision(node_mid):
                cost.append(self.update_cost(node_near, node_mid))
            else:
                cost.append(float("inf"))

        if min(cost) != float('inf'):
            index = int(np.argmin(cost))
            neighbor_min = neighbor_index[index]
            node_new = self.new_state(self.node_list[neighbor_min], node_new)
            node_new.cost = min(cost)
            return node_new

        return None

    def search_best_goal_node(self):
        dist_to_goal_list = [self.dis_to_goal(n) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_len]

        return goal_inds[0]
        # safe_goal_inds = []
        # for goal_ind in goal_inds:
        #     t_node = self.new_state(self.node_list[goal_ind], self.xG)
        #     if self.check_collision(t_node):
        #         safe_goal_inds.append(goal_ind)
        #
        # if not safe_goal_inds:
        #     print('hahhah')
        #     return None
        #
        # min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        # for i in safe_goal_inds:
        #     if self.node_list[i].cost == min_cost:
        #         self.xG.parent = self.node_list[i]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_near = self.node_list[i]
            node_edge = self.new_state(node_new, node_near)
            if not node_edge:
                continue

            node_edge.cost = self.update_cost(node_new, node_near)
            collision = self.check_collision(node_edge)
            improved_cost = node_near.cost > node_edge.cost

            if not collision and improved_cost:
                self.node_list[i] = node_edge
                self.propagate_cost_to_leaves(node_new)

    def update_cost(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)
        return node_start.cost + dist

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.update_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def extract_path(self):
        path = [[self.xG.x, self.xG.y]]
        node = self.xG
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

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
