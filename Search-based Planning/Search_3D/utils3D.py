import numpy as np
import pyrr
from collections import defaultdict


def getRay(x, y):
    direc = [y[0] - x[0], y[1] - x[1], y[2] - x[2]]
    return np.array([x, direc])


def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))


def getManDist(pos1, pos2):
    return sum([abs(pos1[0] - pos2[0]), abs(pos1[1] - pos2[1]), abs(pos1[2] - pos2[2])])


def getNearest(Space, pt):
    '''get the nearest point on the grid'''
    mindis, minpt = 1000, None
    for pts in Space:
        dis = getDist(pts, pt)
        if dis < mindis:
            mindis, minpt = dis, pts
    return minpt


def Heuristic(Space, t):
    '''Max norm distance'''
    h = {}
    for k in Space.keys():
        h[k] = max([abs(t[0] - k[0]), abs(t[1] - k[1]), abs(t[2] - k[2])])
    return h

def heuristic_fun(initparams, k, t=None):
    if t is None:
        t = initparams.goal
    return max([abs(t[0] - k[0]), abs(t[1] - k[1]), abs(t[2] - k[2])])

def isinbound(i, x):
    if i[0] <= x[0] < i[3] and i[1] <= x[1] < i[4] and i[2] <= x[2] < i[5]:
        return True
    return False


def isinball(i, x):
    if getDist(i[0:3], x) <= i[3]:
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
    mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2]  # mid point
    I = [(p1[0] - p0[0]) / dist, (p1[1] - p0[1]) / dist, (p1[2] - p0[2]) / dist]  # unit direction
    hl = dist / 2  # radius
    P = aabb.P  # center of the AABB
    E = aabb.E  # extents of AABB
    T = [P[0] - mid[0], P[1] - mid[1], P[2] - mid[2]]
    # do any of the principal axis form a separting axis?
    if abs(T[0]) > (E[0] + hl * abs(I[0])): return False
    if abs(T[1]) > (E[1] + hl * abs(I[1])): return False
    if abs(T[2]) > (E[2] + hl * abs(I[2])): return False
    # I.cross(x axis) ?
    r = E[1] * abs(I[2]) + E[2] * abs(I[1])
    if abs(T[1] * I[2] - T[2] * I[1]) > r: return False
    # I.cross(y axis) ?
    r = E[0] * abs(I[2]) + E[2] * abs(I[0])
    if abs(T[2] * I[0] - T[0] * I[2]) > r: return False
    # I.cross(z axis) ?
    r = E[0] * abs(I[1]) + E[1] * abs(I[0])
    if abs(T[0] * I[1] - T[1] * I[0]) > r: return False

    return True


def StateSpace(env, factor=0):
    boundary = env.boundary
    resolution = env.resolution
    xmin, xmax = boundary[0] + factor * resolution, boundary[3] - factor * resolution
    ymin, ymax = boundary[1] + factor * resolution, boundary[4] - factor * resolution
    zmin, zmax = boundary[2] + factor * resolution, boundary[5] - factor * resolution
    xarr = np.arange(xmin, xmax, resolution).astype(float)
    yarr = np.arange(ymin, ymax, resolution).astype(float)
    zarr = np.arange(zmin, zmax, resolution).astype(float)
    Space = set()
    for x in xarr:
        for y in yarr:
            for z in zarr:
                Space.add((x, y, z))
    return Space


def g_Space(initparams):
    '''This function is used to get nodes and discretize the space.
       State space is by x*y*z,3 where each 3 is a point in 3D.'''
    g = {}
    Space = StateSpace(initparams.env)
    for v in Space:
        g[v] = np.inf  # this hashmap initialize all g values at inf
    return g


def isCollide(initparams, x, child, dist):
    '''see if line intersects obstacle'''
    '''specified for expansion in A* 3D lookup table'''
    if dist==None:
        dist = getDist(x, child)
    if not isinbound(initparams.env.boundary, child): 
        return True, dist
    for i in range(len(initparams.env.AABB)):
        # if isinbound(initparams.env.blocks[i], child): 
        #     return True, dist
        if lineAABB(x, child, dist, initparams.env.AABB[i]): 
            return True, dist
    for i in initparams.env.balls:
        # if isinball(i, child): 
        #     return True, dist
        if lineSphere(x, child, i): 
            return True, dist
    return False, dist


def children(initparams, x, settings = 0):
    # get the neighbor of a specific state
    allchild = []
    allcost = []
    resolution = initparams.env.resolution
    for direc in initparams.Alldirec:
        child = tuple(map(np.add, x, np.multiply(direc, resolution)))
        if any([isinball(i ,child) for i in initparams.env.balls]):
            continue
        if any([isinbound(i ,child) for i in initparams.env.blocks]):
            continue
        if isinbound(initparams.env.boundary, child):
            allchild.append(child)
            allcost.append((child,initparams.Alldirec[direc]*resolution))
    if settings == 0:
        return allchild
    if settings == 1:
        return allcost


def obstacleFree(initparams, x):
    for i in initparams.env.blocks:
        if isinbound(i, x):
            return False
    for i in initparams.env.balls:
        if isinball(i, x):
            return False
    return True


def cost(initparams, i, j, dist=None, settings='Euclidean'):
    collide, dist = isCollide(initparams, i, j, dist)
    # collide, dist= False, getDist(i, j)
    if settings == 'Euclidean':
        if collide:
            return np.inf
        else:
            return dist
    if settings == 'Manhattan':
        if collide:
            return np.inf
        else:
            return getManDist(i, j)


def initcost(initparams):
    # initialize cost dictionary, could be modifed lateron
    c = defaultdict(lambda: defaultdict(dict))  # two key dicionary
    for xi in initparams.X:
        cdren = children(initparams, xi)
        for child in cdren:
            c[xi][child] = cost(initparams, xi, child)
    return c


if __name__ == "__main__":
    a = '()'
    print(list(a))
