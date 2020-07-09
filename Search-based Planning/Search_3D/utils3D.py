import numpy as np
import pyrr
from collections import defaultdict

def getRay(x, y):
    direc = [y[0] - x[0], y[1] - x[1], y[2] - x[2]]
    return np.array([x, direc])

def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))

def getManDist(pos1, pos2):
    return sum([abs(pos1[0] - pos2[0]),abs(pos1[1] - pos2[1]),abs(pos1[2] - pos2[2])])

def getNearest(Space,pt):
    '''get the nearest point on the grid'''
    mindis,minpt = 1000,None
    for pts in Space: 
        dis = getDist(pts,pt)
        if dis < mindis:
            mindis,minpt = dis,pts
    return minpt

def Heuristic(Space,t):
    '''Max norm distance'''
    h = {}
    for k in Space.keys():
        h[k] = max(abs(np.array([t[0]-k[0],t[1]-k[1],t[2]-k[2]])))
    return h

def hash3D(x):
    return str(x[0])+' '+str(x[1])+' '+str(x[2])

def dehash(x):
    return np.array([float(i) for i in x.split(' ')])

def isinbound(i, x):
    if i[0] <= x[0] < i[3] and i[1] <= x[1] < i[4] and i[2] <= x[2] < i[5]:
        return True
    return False

def isinball(i, x):
    if getDist(i[0:3], x) <= i[3]:
        return True
    return False

def lineSphere(p0,p1,ball):
    # https://cseweb.ucsd.edu/classes/sp19/cse291-d/Files/CSE291_13_CollisionDetection.pdf
    c, r= ball[0:3],ball[-1]
    line = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    d1 = [c[0] - p0[0], c[1] - p0[1], c[2] - p0[2]]
    t = (1 / (line[0]*line[0] + line[1]*line[1] + line[2]*line[2])) * (line[0]*d1[0] + line[1]*d1[1] + line[2]*d1[2])
    if t <= 0: 
        if (d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]) <= r ** 2: return True
    elif t >= 1: 
        d2 = [c[0] - p1[0], c[1] - p1[1], c[2] - p1[2]]
        if (d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]) <= r ** 2: return True
    elif 0 < t < 1: 
        x = [p0[0] + t * line[0], p0[1] + t * line[1], p0[2] + t * line[2]]
        k = [c[0] - x[0], c[1] - x[1], c[2] - x[2]]
        if (k[0] * k[0] + k[1] * k[1] + k[2] * k[2]) <= r**2: return True
    return False
    
def lineAABB(p0,p1,dist,AABB):
    #https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?print=1
    P = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2] # mid point
    D = [(p1[0] - p0[0]) / dist, (p1[1] - p0[1]) / dist, (p1[2] - p0[2]) / dist] # unit direction
    t = dist / 2 # radius
    # TODO: implement this


def StateSpace(env, factor = 0):
    boundary = env.boundary
    resolution = env.resolution
    xmin,xmax = boundary[0]+factor*resolution,boundary[3]-factor*resolution
    ymin,ymax = boundary[1]+factor*resolution,boundary[4]-factor*resolution
    zmin,zmax = boundary[2]+factor*resolution,boundary[5]-factor*resolution
    xarr = np.arange(xmin,xmax,resolution).astype(float)
    yarr = np.arange(ymin,ymax,resolution).astype(float)
    zarr = np.arange(zmin,zmax,resolution).astype(float)
    Space = set()
    for x in xarr:
        for y in yarr:
            for z in zarr:
                Space.add((x,y,z))
    return Space

def g_Space(initparams):
    '''This function is used to get nodes and discretize the space.
       State space is by x*y*z,3 where each 3 is a point in 3D.'''
    g = {}
    Space = StateSpace(initparams.env)
    for v in Space:
        g[v] = np.inf # this hashmap initialize all g values at inf
    return g

def isCollide(initparams, x, child):
    '''see if line intersects obstacle'''
    ray , dist = getRay(x, child) ,  getDist(x, child)
    if not isinbound(initparams.env.boundary,child):
        return True, dist
    for i in initparams.env.AABB:
        shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
        if shot is not None:
            dist_wall = getDist(x, shot)
            if dist_wall <= dist:  # collide
                return True, dist
    for i in initparams.env.balls:
        if isinball(i, child):
            return True, dist
        # shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
        # if shot != []:
        #     dists_ball = [getDist(x, j) for j in shot]
        #     if all(dists_ball <= dist):  # collide
        #         return True, dist
        if lineSphere(x, child, i): return True, dist
    return False, dist

def children(initparams, x):
    # get the neighbor of a specific state
    allchild = []
    resolution = initparams.env.resolution
    for direc in initparams.Alldirec:
        child = tuple(map(np.add,x,np.multiply(direc,resolution)))
        if isinbound(initparams.env.boundary,child):
            allchild.append(child)
    return allchild

def obstacleFree(initparams,x):
    for i in initparams.env.blocks:
        if isinbound(i,x):
            return False
    for i in initparams.env.balls:
        if isinball(i,x):
            return False
    return True

def cost(initparams, i,j,settings=0):
    collide, dist = isCollide(initparams,i,j)
    if settings == 0:
        if collide: return np.inf
        else: return dist
    if settings == 1:
        if collide: return np.inf
        else: return getManDist(i,j)

def initcost(initparams):
    # initialize cost dictionary, could be modifed lateron
    c = defaultdict(lambda: defaultdict(dict)) # two key dicionary
    for xi in initparams.X:
        cdren = children(initparams, xi)
        for child in cdren:
            c[xi][child] = cost(initparams, xi, child)
    return c
    
if __name__ == "__main__":
    a = [10,2.5,1,1]
    lineAABB([0,0,0],[1,1,1],)