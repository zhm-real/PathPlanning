"""
Quintic Polynomial
"""

import math
import numpy as np
import matplotlib.pyplot as plt

import draw


class QuinticPolynomial:
    def __init__(self, x0, v0, a0, x1, v1, a1, T):
        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([x1 - x0 - v0 * T - a0 * T ** 2 / 2,
                      v1 - v0 - a0 * T,
                      a1 - a0])
        X = np.linalg.solve(A, b)

        self.a0 = x0
        self.a1 = v0
        self.a2 = a0 / 2.0
        self.a3 = X[0]
        self.a4 = X[1]
        self.a5 = X[2]

    def calc_xt(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
                self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_dxt(self, t):
        dxt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return dxt

    def calc_ddxt(self, t):
        ddxt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return ddxt

    def calc_dddxt(self, t):
        dddxt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return dddxt


class Trajectory:
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.a = []
        self.jerk = []


def simulation():
    sx, sy, syaw, sv, sa = 10.0, 10.0, np.deg2rad(10.0), 1.0, 0.1
    gx, gy, gyaw, gv, ga = 30.0, -10.0, np.deg2rad(180.0), 1.0, 0.1

    MAX_ACCEL = 1.0  # max accel [m/s2]
    MAX_JERK = 0.5  # max jerk [m/s3]
    dt = 0.1  # T tick [s]

    MIN_T = 5
    MAX_T = 100
    T_STEP = 5

    sv_x = sv * math.cos(syaw)
    sv_y = sv * math.sin(syaw)
    gv_x = gv * math.cos(gyaw)
    gv_y = gv * math.sin(gyaw)

    sa_x = sa * math.cos(syaw)
    sa_y = sa * math.sin(syaw)
    ga_x = ga * math.cos(gyaw)
    ga_y = ga * math.sin(gyaw)

    path = Trajectory()

    for T in np.arange(MIN_T, MAX_T, T_STEP):
        path = Trajectory()
        xqp = QuinticPolynomial(sx, sv_x, sa_x, gx, gv_x, ga_x, T)
        yqp = QuinticPolynomial(sy, sv_y, sa_y, gy, gv_y, ga_y, T)

        for t in np.arange(0.0, T + dt, dt):
            path.t.append(t)
            path.x.append(xqp.calc_xt(t))
            path.y.append(yqp.calc_xt(t))

            vx = xqp.calc_dxt(t)
            vy = yqp.calc_dxt(t)
            path.v.append(np.hypot(vx, vy))
            path.yaw.append(math.atan2(vy, vx))

            ax = xqp.calc_ddxt(t)
            ay = yqp.calc_ddxt(t)
            a = np.hypot(ax, ay)

            if len(path.v) >= 2 and path.v[-1] - path.v[-2] < 0.0:
                a *= -1
            path.a.append(a)

            jx = xqp.calc_dddxt(t)
            jy = yqp.calc_dddxt(t)
            j = np.hypot(jx, jy)

            if len(path.a) >= 2 and path.a[-1] - path.a[-2] < 0.0:
                j *= -1
            path.jerk.append(j)

        if max(np.abs(path.a)) <= MAX_ACCEL and max(np.abs(path.jerk)) <= MAX_JERK:
            break

    print("t_len: ", path.t, "s")
    print("max_v: ", max(path.v), "m/s")
    print("max_a: ", max(np.abs(path.a)), "m/s2")
    print("max_jerk: ", max(np.abs(path.jerk)), "m/s3")

    for i in range(len(path.t)):
        plt.cla()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.axis("equal")
        plt.plot(path.x, path.y, linewidth=2, color='gray')
        draw.Car(sx, sy, syaw, 1.5, 3)
        draw.Car(gx, gy, gyaw, 1.5, 3)
        draw.Car(path.x[i], path.y[i], path.yaw[i], 1.5, 3)
        plt.title("Quintic Polynomial Curves")
        plt.grid(True)
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    simulation()
