import math


def M(theta):
    """
    Return the angle phi = theta mod (2 pi) such that -pi <= theta < pi.
    """
    theta = theta % (2 * math.pi)
    if theta < -math.pi:
        return theta + 2 * math.pi
    if theta >= math.pi:
        return theta - 2 * math.pi
    return theta


def R(x, y):
    """
    Return the polar coordinates (r, theta) of the point (x, y).
    """
    r = math.sqrt(x * x + y * y)
    theta = math.atan2(y, x)
    return r, theta


def change_of_basis(p1, p2):
    """
    Given p1 = (x1, y1, theta1) and p2 = (x2, y2, theta2) represented in a
    coordinate system with origin (0, 0) and rotation 0 (in degrees), return
    the position and rotation of p2 in the coordinate system which origin
    (x1, y1) and rotation theta1.
    """
    theta1 = deg2rad(p1[2])
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    new_x = dx * math.cos(theta1) + dy * math.sin(theta1)
    new_y = -dx * math.sin(theta1) + dy * math.cos(theta1)
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta


def rad2deg(rad):
    return 180 * rad / math.pi


def deg2rad(deg):
    return math.pi * deg / 180


def sign(x):
    return 1 if x >= 0 else -1
