# plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d as plt3d
from mpl_toolkits.mplot3d import proj3d
import numpy as np

def CreateSphere(center,r):
    u = np.linspace(0,2* np.pi,30)
    v = np.linspace(0,np.pi,30)
    x = np.outer(np.cos(u),np.sin(v))
    y = np.outer(np.sin(u),np.sin(v))
    z = np.outer(np.ones(np.size(u)),np.cos(v))
    x, y, z = r*x + center[0], r*y + center[1], r*z + center[2]
    return (x,y,z)

def draw_Spheres(ax,balls):
    for i in balls:
        (xs,ys,zs) = CreateSphere(i[0:3],i[-1])
        ax.plot_wireframe(xs, ys, zs, alpha=0.15,color="b")

def draw_block_list(ax, blocks ,color=None,alpha=0.15):
    '''
    drawing the blocks on the graph
    '''
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
                 dtype='float')
    f = np.array([[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]])
    n = blocks.shape[0]
    d = blocks[:, 3:6] - blocks[:, :3]
    vl = np.zeros((8 * n, 3))
    fl = np.zeros((6 * n, 4), dtype='int64')
    for k in range(n):
        vl[k * 8:(k + 1) * 8, :] = v * d[k] + blocks[k, :3]
        fl[k * 6:(k + 1) * 6, :] = f + k * 8
    if type(ax) is Poly3DCollection:
        ax.set_verts(vl[fl])
    else:
        pc = Poly3DCollection(vl[fl], alpha=alpha, linewidths=1, edgecolors='k')
        pc.set_facecolor(color)
        h = ax.add_collection3d(pc)
        return h

def draw_line(ax,SET,visibility=1,color=None):
    if SET != []:
        for i in SET:
            xs = i[0][0], i[1][0]
            ys = i[0][1], i[1][1]
            zs = i[0][2], i[1][2]
            line = plt3d.art3d.Line3D(xs, ys, zs, alpha=visibility, color=color)
            ax.add_line(line)

def visualization(initparams):
    if initparams.ind % 10 == 0 or initparams.done:
        V = np.array(initparams.V)
        E = initparams.E
        Path = np.array(initparams.Path)
        start = initparams.env.start
        goal = initparams.env.goal
        edges = E.get_edge()
        # generate axis objects
        ax = plt.subplot(111, projection='3d')
        ax.view_init(elev=0., azim=90)
        ax.clear()
        # drawing objects
        draw_Spheres(ax, initparams.env.balls)
        draw_block_list(ax, initparams.env.blocks)
        draw_block_list(ax, np.array([initparams.env.boundary]),alpha=0)
        draw_line(ax,edges,visibility=0.25)
        draw_line(ax,Path,color='r')
        ax.scatter3D(V[:, 0], V[:, 1], V[:, 2], s=2, color='g',)
        ax.plot(start[0:1], start[1:2], start[2:], 'go', markersize=7, markeredgecolor='k')
        ax.plot(goal[0:1], goal[1:2], goal[2:], 'ro', markersize=7, markeredgecolor='k') 
        # adjust the aspect ratio
        xmin, xmax = initparams.env.boundary[0], initparams.env.boundary[3]
        ymin, ymax = initparams.env.boundary[1], initparams.env.boundary[4]
        zmin, zmax = initparams.env.boundary[2], initparams.env.boundary[5]
        dx, dy, dz = xmax-xmin, ymax-ymin, zmax-zmin
        ax.get_proj = make_get_proj(ax,1*dx, 1*dy, 2*dy)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.pause(0.001)

def make_get_proj(self, rx, ry, rz):
    '''
    Return a variation on :func:`~mpl_toolkit.mplot2d.axes3d.Axes3D.getproj` that
    makes the box aspect ratio equal to *rx:ry:rz*, using an axes object *self*.
    '''

    rm = max(rx, ry, rz)
    kx = rm / rx; ky = rm / ry; kz = rm / rz

    # Copied directly from mpl_toolkit/mplot3d/axes3d.py. New or modified lines are
    # marked by ##
    def get_proj():
        relev, razim = np.pi * self.elev/180, np.pi * self.azim/180

        xmin, xmax = self.get_xlim3d()
        ymin, ymax = self.get_ylim3d()
        zmin, zmax = self.get_zlim3d()

        # transform to uniform world coordinates 0-1.0,0-1.0,0-1.0
        worldM = proj3d.world_transformation(xmin, xmax,
                                             ymin, ymax,
                                             zmin, zmax)
        ratio = 0.5
        # adjust the aspect ratio                          ##
        aspectM = proj3d.world_transformation(-kx + 1, kx, ##
                                              -ky + 1, ky, ##
                                              -kz + 1, kz) ##

        # look into the middle of the new coordinates
        R = np.array([0.5, 0.5, 0.5])

        xp = R[0] + np.cos(razim) * np.cos(relev) * self.dist *ratio
        yp = R[1] + np.sin(razim) * np.cos(relev) * self.dist *ratio
        zp = R[2] + np.sin(relev) * self.dist *ratio
        E = np.array((xp, yp, zp))

        self.eye = E
        self.vvec = R - E
        self.vvec = self.vvec / np.linalg.norm(self.vvec)

        if abs(relev) > np.pi/2:
            # upside down
            V = np.array((0, 0, -1))
        else:
            V = np.array((0, 0, 1))
        zfront, zback = -self.dist *ratio, self.dist *ratio

        viewM = proj3d.view_transformation(E, R, V)
        perspM = proj3d.persp_transformation(zfront, zback)
        M0 = np.dot(viewM, np.dot(aspectM, worldM)) ##
        M = np.dot(perspM, M0)
        return M
    return get_proj