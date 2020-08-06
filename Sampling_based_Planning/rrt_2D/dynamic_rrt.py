"""
DYNAMIC_RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"


class Edge:
    def __init__(self, n_p, n_c):
        self.parent = n_p
        self.child = n_c
        self.flag = "VALID"


class DynamicRrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.waypoint_sample_rate = waypoint_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.vertex_old = []
        self.vertex_new = []
        self.edges = []

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()
        self.fig, self.ax = plt.subplots()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        self.obs_add = [0, 0, 0]

        self.path = []
        self.waypoint = []

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)

                    path = self.extract_path(node_new)
                    self.plot_grid("Dynamic_RRT")
                    self.plot_visited()
                    self.plot_path(path)
                    self.path = path
                    self.waypoint = self.extract_waypoint(node_new)
                    self.fig.canvas.mpl_connect('button_press_event', self.on_press)
                    plt.show()

                    return

        return None

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > 50 or y < 0 or y > 30:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Add circle obstacle at: s =", x, ",", "y =", y)
            self.obs_add = [x, y, 2]
            self.obs_circle.append([x, y, 2])
            self.utils.update_obs(self.obs_circle, self.obs_boundary, self.obs_rectangle)
            self.InvalidateNodes()

            if self.is_path_invalid():
                print("Path is Replanning ...")
                path, waypoint = self.replanning()

                print("len_vertex: ", len(self.vertex))
                print("len_vertex_old: ", len(self.vertex_old))
                print("len_vertex_new: ", len(self.vertex_new))

                plt.cla()
                self.plot_grid("Dynamic_RRT")
                self.plot_vertex_old()
                self.plot_path(self.path, color='blue')
                self.plot_vertex_new()
                self.vertex_new = []
                self.plot_path(path)
                self.path = path
                self.waypoint = waypoint
            else:
                print("Trimming Invalid Nodes ...")
                self.TrimRRT()

                plt.cla()
                self.plot_grid("Dynamic_RRT")
                self.plot_visited(animation=False)
                self.plot_path(self.path)

            self.fig.canvas.draw_idle()

    def InvalidateNodes(self):
        for edge in self.edges:
            if self.is_collision_obs_add(edge.parent, edge.child):
                edge.child.flag = "INVALID"

    def is_path_invalid(self):
        for node in self.waypoint:
            if node.flag == "INVALID":
                return True

    def is_collision_obs_add(self, start, end):
        delta = self.utils.delta
        obs_add = self.obs_add

        if math.hypot(start.x - obs_add[0], start.y - obs_add[1]) <= obs_add[2] + delta:
            return True

        if math.hypot(end.x - obs_add[0], end.y - obs_add[1]) <= obs_add[2] + delta:
            return True

        o, d = self.utils.get_ray(start, end)
        if self.utils.is_intersect_circle(o, d, [obs_add[0], obs_add[1]], obs_add[2]):
            return True

        return False

    def replanning(self):
        self.TrimRRT()

        for i in range(self.iter_max):
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                self.vertex_new.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    path = self.extract_path(node_new)
                    waypoint = self.extract_waypoint(node_new)
                    print("path: ", len(path))
                    print("waypoint: ", len(waypoint))

                    return path, waypoint

        return None

    def TrimRRT(self):
        for i in range(1, len(self.vertex)):
            node = self.vertex[i]
            node_p = node.parent
            if node_p.flag == "INVALID":
                node.flag = "INVALID"

        self.vertex = [node for node in self.vertex if node.flag == "VALID"]
        self.vertex_old = copy.deepcopy(self.vertex)
        self.edges = [Edge(node.parent, node) for node in self.vertex[1:len(self.vertex)]]

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def generate_random_node_replanning(self, goal_sample_rate, waypoint_sample_rate):
        delta = self.utils.delta
        p = np.random.random()

        if p < goal_sample_rate:
            return self.s_goal
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:
            return self.waypoint[np.random.randint(0, len(self.waypoint) - 1)]
        else:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def extract_waypoint(self, node_end):
        waypoint = [self.s_goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            waypoint.append(node_now)

        return waypoint

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def plot_grid(self, name):

        for (ox, oy, w, h) in self.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.s_start.x, self.s_start.y, "bs", linewidth=3)
        plt.plot(self.s_goal.x, self.s_goal.y, "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, animation=True):
        if animation:
            count = 0
            for node in self.vertex:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in self.vertex:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_old(self):
        for node in self.vertex_old:
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_new(self):
        count = 0

        for node in self.vertex_new:
            count += 1
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='darkorange')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape' else None])
                if count % 10 == 0:
                    plt.pause(0.001)

    @staticmethod
    def plot_path(path, color='red'):
        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color=color)
        plt.pause(0.01)


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    drrt = DynamicRrt(x_start, x_goal, 0.5, 0.1, 0.6, 5000)
    drrt.planning()


if __name__ == '__main__':
    main()
