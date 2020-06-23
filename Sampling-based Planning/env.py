import numpy as np


class Env:
    def __init__(self):
        self.x_range = (-2, 15)  # size of background
        self.y_range = (-2, 15)
        # self.obs_boundary = self.obs_boundary(self.x_range, self.y_range)
        # self.obs_circle = self.obs_circle()

    @staticmethod
    def obs_boundary(x, y):
        w = 1
        obs_boundary = []

        for i in np.linspace(x[0], x[1], (x[1] - x[0]) // w + 1):
            obs_boundary.append((i, y[0], w))
        for i in np.linspace(x[0], x[1], (x[1] - x[0]) // w + 1):
            obs_boundary.append((i, y[1], w))
        for j in np.linspace(y[0] + 1, y[1] - w, (y[1] - y[0] - 2 * w) // w + 1):
            obs_boundary.append((x[0], j, w))
        for j in np.linspace(y[0] + 1, y[1] - w, (y[1] - y[0] - 2 * w) // w + 1):
            obs_boundary.append((x[1], j, w))

        for i in np.linspace(10, 20, 10 // w + 1):
            obs_boundary.append((i, 15, w))
        for j in np.linspace(1, 14, 13 // w + 1):
            obs_boundary.append((20, j, w))

        for j in np.linspace(15, 29, 14 // w + 1):
            obs_boundary.append((30, j, w))
        for j in np.linspace(1, 14, 13 // w + 1):
            obs_boundary.append((40, j, w))

        return obs_boundary

    @staticmethod
    def obs_circle():
        obs_cir = [
            (8, 8, 3),
            (10, 23, 3),
            (20, 25, 2.5),
            (30, 7, 3),
            (40, 25, 2),
            (43, 20, 2.5)
        ]

        return obs_cir
