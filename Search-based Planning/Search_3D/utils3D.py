import numpy as np
import pyrr

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
    for strpts in Space.keys(): 
        pts = dehash(strpts)
        dis = getDist(pts,pt)
        if dis < mindis:
            mindis,minpt = dis,pts
    return minpt

def Heuristic(Space,t):
    '''Max norm distance'''
    h = {}
    for k in Space.keys():
        h[k] = max(abs(t-dehash(k)))
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
        g[hash3D(v)] = np.inf # this hashmap initialize all g values at inf
    return g

def isCollide(initparams, x, direc):
    '''see if line intersects obstacle'''
    resolution = initparams.env.resolution
    child = np.array(list(map(np.add,x,np.multiply(direc,resolution))))
    ray , dist = getRay(x, child) ,  getDist(x, child)
    if not isinbound(initparams.env.boundary,child):
        return True, child
    for i in initparams.env.AABB:
        shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
        if shot is not None:
            dist_wall = getDist(x, shot)
            if dist_wall <= dist:  # collide
                return True, child
    for i in initparams.env.balls:
        if isinball(i, child):
            return True, child
        shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
        if shot != []:
            dists_ball = [getDist(x, j) for j in shot]
            if all(dists_ball <= dist):  # collide
                return True, child
    return False, child

def obstacleFree(initparams,x):
    for i in initparams.env.blocks:
        if isinbound(i,x):
            return False
    for i in initparams.env.balls:
        if isinball(i,x):
            return False
    return True

def cost(i,j,settings=0):
    if settings == 0:
        return getDist(i,j)
    if settings == 1:
        return getManDist(i,j)
    
if __name__ == "__main__":
    from env3D import env