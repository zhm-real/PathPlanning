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


def sampleFree(initparams, bias = 0.1):
    '''biased sampling'''
    x = np.random.uniform(initparams.env.boundary[0:3], initparams.env.boundary[3:6])
    i = np.random.random()
    if isinside(initparams, x):
        return sampleFree(initparams)
    else:
        if i < bias:
            return initparams.env.goal + 1
        else:
            return x
        return x

# ---------------------- Collision checking algorithms
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

def lineSphere(p0, p1, ball):
    # https://cseweb.ucsd.edu/classes/sp19/cse291-d/Files/CSE291_13_CollisionDetection.pdf
    c, r = ball[0:3], ball[-1]
    line = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    d1 = [c[0] - p0[0], c[1] - p0[1], c[2] - p0[2]]
    t = (1 / (line[0] * line[0] + line[1] * line[1] + line[2] * line[2])) * (
                line[0] * d1[0] + line[1] * d1[1] + line[2] * d1[2])
    if t <= 0:
        if (d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]) <= r ** 2: return True
    elif t >= 1:
        d2 = [c[0] - p1[0], c[1] - p1[1], c[2] - p1[2]]
        if (d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]) <= r ** 2: return True
    elif 0 < t < 1:
        x = [p0[0] + t * line[0], p0[1] + t * line[1], p0[2] + t * line[2]]
        k = [c[0] - x[0], c[1] - x[1], c[2] - x[2]]
        if (k[0] * k[0] + k[1] * k[1] + k[2] * k[2]) <= r ** 2: return True
    return False

def lineAABB(p0, p1, dist, aabb):
    # https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?print=1
    # aabb should have the attributes of P, E as center point and extents
    mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2]  # mid point
    I = [(p1[0] - p0[0]) / dist, (p1[1] - p0[1]) / dist, (p1[2] - p0[2]) / dist]  # unit direction
    hl = dist / 2  # radius
    T = [aabb.P[0] - mid[0], aabb.P[1] - mid[1], aabb.P[2] - mid[2]]
    # do any of the principal axis form a separting axis?
    if abs(T[0]) > (aabb.E[0] + hl * abs(I[0])): return False
    if abs(T[1]) > (aabb.E[1] + hl * abs(I[1])): return False
    if abs(T[2]) > (aabb.E[2] + hl * abs(I[2])): return False
    # I.cross(x axis) ?
    r = aabb.E[1] * abs(I[2]) + aabb.E[2] * abs(I[1])
    if abs(T[1] * I[2] - T[2] * I[1]) > r: return False
    # I.cross(y axis) ?
    r = aabb.E[0] * abs(I[2]) + aabb.E[2] * abs(I[0])
    if abs(T[2] * I[0] - T[0] * I[2]) > r: return False
    # I.cross(z axis) ?
    r = aabb.E[0] * abs(I[1]) + aabb.E[1] * abs(I[0])
    if abs(T[0] * I[1] - T[1] * I[0]) > r: return False

    return True

def lineOBB(p0, p1, dist, obb):
    # transform points to obb frame
    res = obb.T@np.column_stack([np.array([p0,p1]),[1,1]]).T 
    # record old position and set the position to origin
    oldP, obb.P= obb.P, [0,0,0] 
    # calculate segment-AABB testing
    ans = lineAABB(res[0:3,0],res[0:3,1],dist,obb)
    # reset the position
    obb.P = oldP 
    return ans

def isCollide(initparams, x, child, dist=None):
    '''see if line intersects obstacle'''
    '''specified for expansion in A* 3D lookup table'''
    if dist==None:
        dist = getDist(x, child)
    # check in bound
    if not isinbound(initparams.env.boundary, child): 
        return True, dist
    # check collision in AABB
    for i in range(len(initparams.env.AABB)):
        if lineAABB(x, child, dist, initparams.env.AABB[i]): 
            return True, dist
    # check collision in ball
    for i in initparams.env.balls:
        if lineSphere(x, child, i): 
            return True, dist
    # check collision with obb
    for i in initparams.env.OBB:
        if lineOBB(x, child, dist, i):
            return True, dist
    return False, dist

# ---------------------- leaf node extending algorithms
def nearest(initparams, x):
    V = np.array(initparams.V)
    if initparams.i == 0:
        return initparams.V[0]
    xr = repmat(x, len(V), 1)
    dists = np.linalg.norm(xr - V, axis=1)
    return tuple(initparams.V[np.argmin(dists)])

def near(initparams, x):
    x = np.array(x)
    V = np.array(initparams.V)
    cardV = len(initparams.V)
    eta = initparams.eta
    gamma = initparams.gamma
    r = min(gamma * (np.log(cardV) / cardV), eta)
    if initparams.done: 
        r = 1
    if initparams.i == 0:
        return [initparams.V[0]]
    xr = repmat(x, len(V), 1)
    inside = np.linalg.norm(xr - V, axis=1) < r
    nearpoints = V[inside]
    return np.array(nearpoints)

def steer(initparams, x, y):
    dist, step = getDist(y, x), initparams.stepsize
    increment = ((y[0] - x[0]) / dist * step, (y[1] - x[1]) / dist * step, (y[2] - x[2]) / dist * step)
    xnew = (x[0] + increment[0], x[1] + increment[1], x[2] + increment[2])
    # direc = (y - x) / np.linalg.norm(y - x)
    # xnew = x + initparams.stepsize * direc
    return xnew

def cost(initparams, x):
    '''here use the additive recursive cost function'''
    if x == initparams.x0:
        return 0
    return cost(initparams, initparams.Parent[x]) + getDist(x, initparams.Parent[x])


def path(initparams, Path=[], dist=0):
    x = tuple(initparams.env.goal)
    while x != tuple(initparams.env.start):
        x2 = initparams.Parent[x]
        Path.append(np.array([x, x2]))
        dist += getDist(x, x2)
        x = x2
    return Path, dist

class edgeset(object):
    def __init__(self):
        self.E = {}

    def add_edge(self, edge):
        x, y = edge[0], edge[1]
        if x in self.E:
            self.E[x].add(y)
        else:
            self.E[x] = set()
            self.E[x].add(y)

    def remove_edge(self, edge):
        x, y = edge[0], edge[1]
        self.E[x].remove(y)

    def get_edge(self):
        edges = []
        for v in self.E:
            for n in self.E[v]:
                # if (n,v) not in edges:
                edges.append((v, n))
        return edges
