import numpy as np
from numpy.matlib import repmat
import pyrr as pyrr

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Sampling-based Planning/")
from rrt_3D.plot_util3D import visualization


def getRay(x, y):
    direc = [y[0] - x[0], y[1] - x[1], y[2] - x[2]]
    return np.array([x, direc])


def getAABB(blocks):
    AABB = []
    for i in blocks:
        AABB.append(np.array([np.add(i[0:3], -0), np.add(i[3:6], 0)]))  # make AABBs alittle bit of larger
    return AABB


def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))


''' The following utils can be used for rrt or rrt*,
    required param initparams should have
    env,      environement generated from env3D
    V,        node set
    E,        edge set
    i,        nodes added
    maxiter,  maximum iteration allowed
    stepsize, leaf growth restriction

'''


def sampleFree(initparams):
    '''biased sampling'''
    x = np.random.uniform(initparams.env.boundary[0:3], initparams.env.boundary[3:6])
    i = np.random.random()
    if isinside(initparams, x):
        return sampleFree(initparams)
    else:
        if i < 0.1:
            return initparams.env.goal + 1
        else:
            return np.array(x)
        return np.array(x)


def isinside(initparams, x):
    '''see if inside obstacle'''
    for i in initparams.env.blocks:
        if i[0] <= x[0] < i[3] and i[1] <= x[1] < i[4] and i[2] <= x[2] < i[5]:
            return True
    return False


def isinbound(i, x):
    if i[0] <= x[0] < i[3] and i[1] <= x[1] < i[4] and i[2] <= x[2] < i[5]:
        return True
    return False


def isCollide(initparams, x, y):
    '''see if line intersects obstacle'''
    ray = getRay(x, y)
    dist = getDist(x, y)
    if not isinbound(initparams.env.boundary, y):
        return True
    for i in getAABB(initparams.env.blocks):
        shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
        if shot is not None:
            dist_wall = getDist(x, shot)
            if dist_wall <= dist:  # collide
                return True
    for i in initparams.env.balls:
        shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
        if shot != []:
            dists_ball = [getDist(x, j) for j in shot]
            if all(dists_ball <= dist):  # collide
                return True
    return False


def nearest(initparams, x):
    V = np.array(initparams.V)
    if initparams.i == 0:
        return initparams.V[0]
    xr = repmat(x, len(V), 1)
    dists = np.linalg.norm(xr - V, axis=1)
    return initparams.V[np.argmin(dists)]


def steer(initparams, x, y):
    direc = (y - x) / np.linalg.norm(y - x)
    xnew = x + initparams.stepsize * direc
    return xnew


def near(initparams, x):
    cardV = len(initparams.V)
    eta = initparams.eta
    gamma = initparams.gamma
    r = min(gamma * (np.log(cardV) / cardV), eta)
    if initparams.done: r = 1
    V = np.array(initparams.V)
    if initparams.i == 0:
        return [initparams.V[0]]
    xr = repmat(x, len(V), 1)
    inside = np.linalg.norm(xr - V, axis=1) < r
    nearpoints = V[inside]
    return np.array(nearpoints)


def cost(initparams, x):
    '''here use the additive recursive cost function'''
    if all(x == initparams.env.start):
        return 0
    xparent = initparams.Parent[hash3D(x)]
    return cost(initparams, xparent) + getDist(x, xparent)


def path(initparams, Path=[], dist=0):
    x = initparams.env.goal
    while not all(x == initparams.env.start):
        x2 = initparams.Parent[hash3D(x)]
        Path.append(np.array([x, x2]))
        dist += getDist(x, x2)
        x = x2
    return Path, dist


def hash3D(x):
    return str(x[0]) + ' ' + str(x[1]) + ' ' + str(x[2])


def dehash(x):
    return np.array([float(i) for i in x.split(' ')])


class edgeset(object):
    def __init__(self):
        self.E = {}

    def add_edge(self, edge):
        x, y = hash3D(edge[0]), hash3D(edge[1])
        if x in self.E:
            self.E[x].append(y)
        else:
            self.E[x] = [y]

    def remove_edge(self, edge):
        x, y = edge[0], edge[1]
        self.E[hash3D(x)].remove(hash3D(y))

    def get_edge(self):
        edges = []
        for v in self.E:
            for n in self.E[v]:
                # if (n,v) not in edges:
                edges.append((dehash(v), dehash(n)))
        return edges
