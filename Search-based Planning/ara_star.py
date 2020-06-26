import queue
import plotting
import env

import matplotlib.pyplot as plt


class AraStar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.e = 3
        self.g = {self.xI: 0, self.xG: float("inf")}
        self.fig_name = "ARA_Star Algorithm"

        self.OPEN = queue.QueuePrior()  # priority queue / OPEN
        self.CLOSED = []
        self.INCONS = []
        self.parent = {self.xI: self.xI}

    def searching(self):
        path = []

        self.OPEN.put(self.xI, self.fvalue(self.xI))
        self.ImprovePath()

        path.append(self.extract_path())

        while self.update_e() > 1:
            self.e -= 0.5
            OPEN_mid = [x for (p, x) in self.OPEN.enumerate()] + self.INCONS
            self.OPEN = queue.QueuePrior()

            for x in OPEN_mid:
                self.OPEN.put(x, self.fvalue(x))
            self.INCONS = []
            self.CLOSED = []
            self.ImprovePath()

            path.append(self.extract_path())

        return path

    def ImprovePath(self):
        while (not self.OPEN.empty() and self.fvalue(self.xG) >
               min([self.fvalue(x) for (p, x) in self.OPEN.enumerate()])):
            s = self.OPEN.get()
            self.CLOSED.append(s)

            for u_next in self.u_set:
                s_next = tuple([s[i] + u_next[i] for i in range(len(s))])
                if s_next not in self.obs:
                    new_cost = self.g[s] + self.get_cost(s, u_next)
                    if s_next not in self.g or new_cost < self.g[s_next]:
                        self.g[s_next] = new_cost
                        self.parent[s_next] = s
                        if s_next not in self.CLOSED:
                            self.OPEN.put(s_next, self.fvalue(s_next))
                        else:
                            self.INCONS.append(s_next)

    def update_e(self):
        c_OPEN, c_INCONS = float("inf"), float("inf")

        if not self.OPEN.empty():
            c_OPEN = min(self.g[x] + self.Heuristic(x) for (p, x) in self.OPEN.enumerate())

        if len(self.INCONS) != 0:
            c_INCONS = min(self.g[x] + self.Heuristic(x) for x in self.INCONS)

        if min(c_OPEN, c_INCONS) == float("inf"):
            return 1
        else:
            return min(self.e, self.g[self.xG] / min(c_OPEN, c_INCONS))

    def fvalue(self, x):
        h = self.e * self.Heuristic(x)
        return self.g[x] + h

    def extract_path(self):
        """
        Extract the path based on the relationship of nodes.

        :param policy: Action needed for transfer between two nodes
        :return: The planning path
        """

        path_back = [self.xG]
        x_current = self.xG

        while True:
            x_current = self.parent[x_current]
            path_back.append(x_current)

            if x_current == self.xI:
                break

        return list(path_back)

    @staticmethod
    def get_cost(x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1

    def Heuristic(self, state):
        """
        Calculate heuristic.
        :param state: current node (state)
        :param goal: goal node (state)
        :param heuristic_type: choosing different heuristic functions
        :return: heuristic
        """

        heuristic_type = self.heuristic_type
        goal = self.xG

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


def main():
    x_start = (5, 5)  # Starting node
    x_goal = (49, 5)  # Goal node

    arastar = AraStar(x_start, x_goal, "manhattan")
    plot = plotting.Plotting(x_start, x_goal)

    path = arastar.searching()

    plot.plot_grid("ARA*")

    print(arastar.e)

    for path_i in path:
        path_i.remove(x_start)
        path_i.remove(x_goal)

        path_x = [path_i[i][0] for i in range(len(path_i))]
        path_y = [path_i[i][1] for i in range(len(path_i))]

        plt.plot(path_x, path_y, linewidth='3', marker='o')
        plt.pause(1)

    plt.show()


if __name__ == '__main__':
    main()
