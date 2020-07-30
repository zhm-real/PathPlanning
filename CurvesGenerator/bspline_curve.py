"""

Path Planner with B-Spline

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate
import cubic_spline as cs


def approximate_b_spline_path(x, y, n_path_points, degree=3):
    t = range(len(x))
    x_tup = scipy_interpolate.splrep(t, x, k=degree)
    y_tup = scipy_interpolate.splrep(t, y, k=degree)

    x_list = list(x_tup)
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    rx = scipy_interpolate.splev(ipl_t, x_list)
    ry = scipy_interpolate.splev(ipl_t, y_list)

    return rx, ry


def interpolate_b_spline_path(x, y, n_path_points, degree=3):
    ipl_t = np.linspace(0.0, len(x) - 1, len(x))
    spl_i_x = scipy_interpolate.make_interp_spline(ipl_t, x, k=degree)
    spl_i_y = scipy_interpolate.make_interp_spline(ipl_t, y, k=degree)

    travel = np.linspace(0.0, len(x) - 1, n_path_points)
    return spl_i_x(travel), spl_i_y(travel)


def main():
    print(__file__ + " start!!")
    # way points
    # way_point_x = [-1.0, 3.0, 4.0, 2.0, 1.0]
    # way_point_y = [0.0, -3.0, 1.0, 1.0, 3.0]
    way_point_x = [-2, 2.0, 3.5, 5.5, 6.0, 8.0]
    way_point_y = [0, 2.7, -0.5, 0.5, 3.0, 4.0]

    sp = cs.Spline2D(way_point_x, way_point_y)
    s = np.arange(0, sp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    n_course_point = 100  # sampling number
    rax, ray = approximate_b_spline_path(way_point_x, way_point_y,
                                         n_course_point)
    rix, riy = interpolate_b_spline_path(way_point_x, way_point_y,
                                         n_course_point)

    # show results
    plt.plot(way_point_x, way_point_y, '-og', label="Control Points")
    plt.plot(rax, ray, '-r', label="Approximated B-Spline path")
    plt.plot(rix, riy, '-b', label="Interpolated B-Spline path")
    plt.plot(rx, ry, color='dimgray', label="Cubic Spline")
    plt.grid(True)
    plt.title("Curves Comparison")
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
