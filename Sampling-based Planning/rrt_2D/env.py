class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            (0, 0, 1, 30),
            (0, 30, 50, 1),
            (1, 0, 50, 1),
            (50, 1, 1, 30)
            # (20, 1, 1, 15),
            # (10, 15, 10, 1),
            # (30, 15, 1, 15),
            # (40, 1, 1, 15)
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            (13, 10, 5, 3),
            (18, 4, 5, 4),
            (22, 13, 6, 3),
            (33, 15, 5, 3),
            (42, 6, 5, 3)
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            (5, 10, 3),
            (10, 22, 3.5),
            (21, 23, 3),
            (34, 9, 4),
            (37, 23, 3),
            (45, 20, 2)
        ]

        return obs_cir
