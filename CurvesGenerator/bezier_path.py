"""
bezier path

author: Atsushi Sakai(@Atsushi_twi)
modified: huiming zhou
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb
import draw


def calc_4points_bezier_path(sx, sy, syaw, gx, gy, gyaw, offset):

    dist = np.hypot(sx - gx, sy - gy) / offset
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [gx - dist * np.cos(gyaw), gy - dist * np.sin(gyaw)],
         [gx, gy]])

    path = calc_bezier_path(control_points, n_points=100)

    return path, control_points


def calc_bezier_path(control_points, n_points=100):
    traj = []

    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)


def Comb(n, i, t):
    return comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    n = len(control_points) - 1
    return np.sum([Comb(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    w = {0: control_points}

    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                             for j in range(n - 1)])

    return w


def curvature(dx, dy, ddx, ddy):
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


def simulation():
    sx = [-3, 0, 4, 6]
    sy = [2, 0, 1.5, 6]

    ratio = np.linspace(0, 1, 100)
    pathx, pathy = [], []

    for t in ratio:
        x, y = [], []
        for i in range(len(sx) - 1):
            x.append(sx[i + 1] * t + sx[i] * (1 - t))
            y.append(sy[i + 1] * t + sy[i] * (1 - t))

        xx, yy = [], []
        for i in range(len(x) - 1):
            xx.append(x[i + 1] * t + x[i] * (1 - t))
            yy.append(y[i + 1] * t + y[i] * (1 - t))

        px = xx[1] * t + xx[0] * (1 - t)
        py = yy[1] * t + yy[0] * (1 - t)
        pathx.append(px)
        pathy.append(py)

        plt.cla()
        plt.plot(sx, sy, linestyle='-', marker='o', color='dimgray', label="Control Points")
        plt.plot(x, y, color='dodgerblue')
        plt.plot(xx, yy, color='cyan')
        plt.plot(pathx, pathy, color='darkorange', linewidth=2, label="Bezier Path")
        plt.plot(px, py, marker='o')
        plt.axis("equal")
        plt.legend()
        plt.title("Cubic Bezier Curve demo")
        plt.grid(True)
        plt.pause(0.001)

    plt.show()


def main():
    sx, sy, syaw = 10.0, 1.0, np.deg2rad(180.0)
    gx, gy, gyaw = 0.0, -3.0, np.deg2rad(-45.0)
    offset = 3.0

    path, control_points = calc_4points_bezier_path(sx, sy, syaw, gx, gy, gyaw, offset)

    t = 0.8  # Number in [0, 1]
    x_target, y_target = bezier(t, control_points)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(t, control_points)
    dt = bezier(t, derivatives_cp[1])
    ddt = bezier(t, derivatives_cp[2])
    # Radius of curv
    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius,
                        color=(0, 0.8, 0.8), fill=False, linewidth=1)

    assert path.T[0][0] == sx, "path is invalid"
    assert path.T[1][0] == sy, "path is invalid"
    assert path.T[0][-1] == gx, "path is invalid"
    assert path.T[1][-1] == gy, "path is invalid"

    fig, ax = plt.subplots()
    ax.plot(path.T[0], path.T[1], label="Bezier Path")
    ax.plot(control_points.T[0], control_points.T[1],
            '--o', label="Control Points")
    ax.plot(x_target, y_target)
    ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
    ax.plot(normal[:, 0], normal[:, 1], label="Normal")
    ax.add_artist(circle)
    draw.Arrow(sx, sy, syaw, 1, "darkorange")
    draw.Arrow(gx, gy, gyaw, 1, "darkorange")
    plt.grid(True)
    plt.title("Bezier Path: from Atsushi's work")
    ax.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
    # simulation()
