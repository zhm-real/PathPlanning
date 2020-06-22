import env
import plotting
import motion_model

import numpy as np
import sys


class Value_iteration:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.e = 0.001  # threshold for convergence
        self.gamma = 0.9  # discount factor

        self.env = env.Env(self.xI, self.xG)  # class Env
        self.motion = motion_model.Motion_model(self.xI, self.xG)  # class Motion_model
        self.plotting = plotting.Plotting(self.xI, self.xG)  # class Plotting

        self.u_set = self.env.motions  # feasible input set
        self.stateSpace = self.env.stateSpace  # state space
        self.obs = self.env.obs_map()  # position of obstacles
        self.lose = self.env.lose_map()  # position of lose states

        self.name1 = "value_iteration, gamma=" + str(self.gamma)
        self.name2 = "converge process, e=" + str(self.e)

        [self.value, self.policy, self.diff] = self.iteration(self.xI, self.xG)
        self.path = self.extract_path(self.xI, self.xG, self.policy)
        self.plotting.animation(self.path, self.name1)
        self.plotting.plot_diff(self.diff, self.name2)

    def iteration(self, xI, xG):
        """
        value_iteration.

        :return: converged value table, optimal policy and variation of difference,
        """

        value_table = {}  # value table
        policy = {}  # policy
        diff = []  # maximum difference between two successive iteration
        delta = sys.maxsize  # initialize maximum difference
        count = 0  # iteration times

        for x in self.stateSpace:  # initialize value table for feasible states
            value_table[x] = 0

        while delta > self.e:  # converged condition
            count += 1
            x_value = 0
            for x in self.stateSpace:
                if x not in xG:
                    value_list = []
                    for u in self.u_set:
                        [x_next, p_next] = self.motion.move_next(x, u)  # recall motion model
                        value_list.append(self.cal_Q_value(x_next, p_next, value_table))  # cal Q value
                    policy[x] = self.u_set[int(np.argmax(value_list))]  # update policy
                    v_diff = abs(value_table[x] - max(value_list))  # maximum difference
                    value_table[x] = max(value_list)  # update value table
                    x_value = max(x_value, v_diff)
            delta = x_value  # update delta
            diff.append(delta)

        self.message(count)  # print messages

        return value_table, policy, diff

    def cal_Q_value(self, x, p, table):
        """
        cal Q_value.

        :param x: next state vector
        :param p: probability of each state
        :param table: value table
        :return: Q-value
        """

        value = 0
        reward = self.env.get_reward(x)  # get reward of next state
        for i in range(len(x)):
            value += p[i] * (reward[i] + self.gamma * table[x[i]])  # cal Q-value

        return value

    def extract_path(self, xI, xG, policy):
        """
        extract path from converged policy.

        :param xI: starting state
        :param xG: goal states
        :param policy: converged policy
        :return: path
        """

        x, path = xI, [xI]
        while x not in xG:
            u = policy[x]
            x_next = (x[0] + u[0], x[1] + u[1])
            if x_next in self.obs:
                print("Collision! Please run again!")
                break
            else:
                path.append(x_next)
                x = x_next
        return path

    def message(self, count):
        """
        print important message.

        :param count: iteration numbers
        :return: print
        """

        print("starting state: ", self.xI)
        print("goal states: ", self.xG)
        print("condition for convergence: ", self.e)
        print("discount factor: ", self.gamma)
        print("iteration times: ", count)


if __name__ == '__main__':
    x_Start = (5, 5)  # starting state
    x_Goal = [(49, 5), (49, 25)]  # goal states

    VI = Value_iteration(x_Start, x_Goal)
