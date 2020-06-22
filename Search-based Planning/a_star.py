import queue
import plotting
import env


class Astar:
    def __init__(self, x_start, x_goal, heuristic_type):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()  # class Env
        self.plotting = plotting.Plotting(self.xI, self.xG)  # class Plotting

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        [self.path, self.policy, self.visited] = self.searching(self.xI, self.xG, heuristic_type)

        self.fig_name = "A* Algorithm"
        self.plotting.animation(self.path, self.visited, self.fig_name)  # animation generate

    def searching(self, xI, xG, heuristic_type):
        """
        Searching using A_star.

        :return: planning path, action in each node, visited nodes in the planning process
        """

        q_astar = queue.QueuePrior()  # priority queue
        q_astar.put(xI, 0)
        parent = {xI: xI}  # record parents of nodes
        action = {xI: (0, 0)}  # record actions of nodes
        visited = []
        cost = {xI: 0}

        while not q_astar.empty():
            x_current = q_astar.get()
            if x_current == xG:  # stop condition
                break
            visited.append(x_current)
            for u_next in self.u_set:  # explore neighborhoods of current node
                x_next = tuple([x_current[i] + u_next[i] for i in range(len(x_current))])
                if x_next not in self.obs:
                    new_cost = cost[x_current] + self.get_cost(x_current, u_next)
                    if x_next not in cost or new_cost < cost[x_next]:  # conditions for updating cost
                        cost[x_next] = new_cost
                        priority = new_cost + self.Heuristic(x_next, xG, heuristic_type)
                        q_astar.put(x_next, priority)  # put node into queue using priority "f+h"
                        parent[x_next], action[x_next] = x_current, u_next

        [path, policy] = self.extract_path(xI, xG, parent, action)

        return path, policy, visited

    def extract_path(self, xI, xG, parent, policy):
        """
        Extract the path based on the relationship of nodes.

        :param xI: Starting node
        :param xG: Goal node
        :param parent: Relationship between nodes
        :param policy: Action needed for transfer between two nodes
        :return: The planning path
        """

        path_back = [xG]
        acts_back = [policy[xG]]
        x_current = xG
        while True:
            x_current = parent[x_current]
            path_back.append(x_current)
            acts_back.append(policy[x_current])
            if x_current == xI: break

        return list(path_back), list(acts_back)

    def get_cost(self, x, u):
        """
        Calculate cost for this motion

        :param x: current node
        :param u: input
        :return:  cost for this motion
        :note: cost function could be more complicate!
        """

        return 1

    def Heuristic(self, state, goal, heuristic_type):
        """
        Calculate heuristic.

        :param state: current node (state)
        :param goal: goal node (state)
        :param heuristic_type: choosing different heuristic functions
        :return: heuristic
        """

        if heuristic_type == "manhattan":
            return abs(goal[0] - state[0]) + abs(goal[1] - state[1])
        elif heuristic_type == "euclidean":
            return ((goal[0] - state[0]) ** 2 + (goal[1] - state[1]) ** 2) ** (1 / 2)
        else:
            print("Please choose right heuristic type!")


if __name__ == '__main__':
    x_Start = (5, 5)  # Starting node
    x_Goal = (49, 5)  # Goal node

    astar = Astar(x_Start, x_Goal, "manhattan")
