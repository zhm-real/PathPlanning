import numpy as np

class Env:
    def __init__(self):
        self.x_range = (0, 50)  # size of background
        self.y_range = (0, 30)
        self.obs = self.obs_map()

    def obs_map(self):
        """
        Initialize obstacles' positions

        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        w = 2

        obs_boundary = []

        for i in np.linspace(x[0], x[1], (x[1]-x[0])//w+1):
            obs_boundary.append((i, y[0], w))
        for i in np.linspace(x[0], x[1], (x[1]-x[0])//w+1):
            obs_boundary.append((i, y[1], w))
        for j in np.linspace(y[0], y[1], (y[1]-y[0])//w+1):
            obs_boundary.append((j, x[0], w))
        for j in np.linspace(y[0], y[1], (y[1]-y[0])//w+1):
            obs_boundary.append((j, x[1], w))

