import queue
import plotting
import env


class AraStar:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles



def main():
    x_start = (5, 5)  # Starting node
    x_goal = (49, 5)  # Goal node

    arastar = AraStar(x_start, x_goal)
