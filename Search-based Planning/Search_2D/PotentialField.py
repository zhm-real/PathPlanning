"""
Potential_Field
@author: huiming zhou
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search-based Planning/")

import
from Search_2D import plotting
from Search_2D import env


class PF:
    def __init__(self, s_start, s_goal):
        self.s_start = s_start
        self.s_goal = s_goal
        self.kp = 5.0
        self.eta = 400
        self.r = 30.0
        self.OL = 10
        self.rr = 2

        self.Env = env.Env()
        self.Plot = plotting.Plotting(s_start, s_goal)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

    def run(self):
        pmap, minx, miny = self.calc_potential_field()

        d = np.hypot(self.s_start[0] - self.s_goal[0],
                     self.s_start[1] - self.s_goal[1])

        ix = self.s_start[0] - minx
        iy = self.s_start[1] - miny
        gix = self.s_goal[0] - minx
        giy = self.s_goal[1] - miny

        self.draw_heatmap(pmap)
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "sk")
        plt.plot(gix, giy, "sm")

        rx, ry = [self.s_start[0]], [self.s_goal[0]]
        ids_rec = deque()

        while d >= 1:
            minp = float("inf")
            minix, miniy = -1, -1

            for u in self.u_set:
                inx = int(ix + u[0])
                iny = int(iy + u[1])

                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")
                    print("test")
                else:
                    p = pmap[inx][iny]

                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny

            ix = minix
            iy = miniy
            xp = ix + minx
            yp = iy + miny
            d = np.hypot(self.s_goal[0] - xp, self.s_goal[1] - yp)
            rx.append(xp)
            ry.append(yp)

            if self.is_oscillation(ids_rec, ix, iy):
                print("Oscillation detected at ({},{})!".format(ix, iy))
                break

            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

        return rx, ry

    def calc_potential_field(self):
        minx = 0
        miny = 0
        maxx = self.x - 1
        maxy = self.y - 1
        xw = maxx - minx
        yw = maxy - miny

        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix + minx

            for iy in range(yw):
                y = iy + miny
                ug = self.calc_attractive_potential(x, y)
                uo = self.calc_repulsive_potential(x, y)
                uf = ug + uo
                pmap[ix][iy] = uf

        return pmap, minx, miny

    def calc_attractive_potential(self, x, y):
        return 0.5 * self.kp * \
               np.hypot(x - self.s_goal[0], y - self.s_goal[1])

    def calc_repulsive_potential(self, x, y):
        dmin = float("inf")

        for s in self.obs:
            d = np.hypot(x - s[0], y - s[1])
            if dmin >= d:
                dmin = d

        # calc repulsive potential
        dq = dmin

        if dq <= self.rr:
            if dq <= 0.8:
                dq = 0.8

            return 0.5 * self.eta * (1.0 / dq - 1.0 / self.rr) ** 2
        else:
            return 0.0

    def is_oscillation(self, ids_rec, ix, iy):
        ids_rec.append((ix, iy))

        if len(ids_rec) > self.OL:
            ids_rec.popleft()

        ids_set = set()
        for s in ids_rec:
            if s in ids_set:
                return True
            else:
                ids_set.add(s)
        return False

    @staticmethod
    def draw_heatmap(data):
        data = np.array(data).T
        plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    pf = PF(s_start, s_goal)

    plt.grid(True)
    plt.axis("equal")
    _, _ = pf.run()
    plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
