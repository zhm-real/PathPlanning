class Env:
    def __init__(self):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        self.obs = self.obs_map()

    def obs_map(self):
        """
        Initialize obstacles' positions

        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = []

        for i in range(x):
            obs.append((i, 0))
        for i in range(x):
            obs.append((i, y - 1))

        for i in range(y):
            obs.append((0, i))
        for i in range(y):
            obs.append((x - 1, i))

        for i in range(10, 21):
            obs.append((i, 15))
        for i in range(15):
            obs.append((20, i))

        for i in range(15, 30):
            obs.append((30, i))
        for i in range(16):
            obs.append((40, i))

        return obs
