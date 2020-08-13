import numpy as np
import pyrr
from collections import defaultdict
import copy


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

def isinbound(i, x, mode = False, factor = 0, isarray = False):
    if mode == 'obb':
        return isinobb(i, x, isarray)
    if isarray:
        compx = (i[0] - factor <= x[:,0]) & (x[:,0] < i[3] + factor) 
        compy = (i[1] - factor <= x[:,1]) & (x[:,1] < i[4] + factor) 
        compz = (i[2] - factor <= x[:,2]) & (x[:,2] < i[5] + factor) 
        return compx & compy & compz
    else:    
        return i[0] - factor <= x[0] < i[3] + factor and i[1] - factor <= x[1] < i[4] + factor and i[2] - factor <= x[2] < i[5] + factor

def isinball(i, x, factor = 0):
    if getDist(i[0:3], x) <= i[3] + factor:
        return True
    return False

def isinobb(i, x, isarray = False):
    # transform the point from {W} to {body}
    if isarray:
        pts = (i.T@np.column_stack((x, np.ones(len(x)))).T).T[:,0:3]
        block = [- i.E[0],- i.E[1],- i.E[2],+ i.E[0],+ i.E[1],+ i.E[2]]
        return isinbound(block, pts, isarray = isarray)
    else:
        pt = i.T@np.append(x,1)
        block = [- i.E[0],- i.E[1],- i.E[2],+ i.E[0],+ i.E[1],+ i.E[2]]
        return isinbound(block, pt)

def OBB2AABB(obb):
    # https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?print=1
    aabb = copy.deepcopy(obb)
    P = obb.P
    a = obb.E
    A = obb.O
    # a1(A1 dot s) + a2(A2 dot s) + a3(A3 dot s)
    Ex = a[0]*abs(A[0][0]) + a[1]*abs(A[1][0]) + a[2]*abs(A[2][0])
    Ey = a[0]*abs(A[0][1]) + a[1]*abs(A[1][1]) + a[2]*abs(A[2][1])
    Ez = a[0]*abs(A[0][2]) + a[1]*abs(A[1][2]) + a[2]*abs(A[2][2])
    E = np.array([Ex, Ey, Ez])
    aabb.P = P
    aabb.E = E
    aabb.O = np.array([[1,0,0],[0,1,0],[0,0,1]])
    return aabb

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
    # I.cross(s axis) ?
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

def OBBOBB(obb1, obb2):
    # https://www.gamasutra.com/view/feature/131790/simple_intersection_tests_for_games.php?print=1
    # each obb class should contain attributes:
    # E: extents along three principle axis in R3
    # P: position of the center axis in R3
    # O: orthornormal basis in R3*3
    a , b = np.array(obb1.E), np.array(obb2.E)
    Pa, Pb = np.array(obb1.P), np.array(obb2.P)
    A , B = np.array(obb1.O), np.array(obb2.O)
    # check if two oriented bounding boxes overlap
    # translation, in parent frame
    v = Pb - Pa
    # translation, in A's frame
    # vdotA[0],vdotA[1],vdotA[2]
    T = [v@B[0], v@B[1], v@B[2]]
    R = np.zeros([3,3])
    for i in range(0,3):
        for k in range(0,3):
            R[i][k] = A[i]@B[k]
            # use separating axis thm for all 15 separating axes
            # if the separating axis cannot be found, then overlap
            # A's basis vector
            for i in range(0,3):
                ra = a[i]
                rb = b[0]*abs(R[i][0]) + b[1]*abs(R[i][1]) + b[2]*abs(R[i][2])
                t = abs(T[i])
                if t > ra + rb:
                    return False
            for k in range(0,3):
                ra = a[0]*abs(R[0][k]) + a[1]*abs(R[1][k]) + a[2]*abs(R[2][k])
                rb = b[k]
                t = abs(T[0]*R[0][k] + T[1]*R[1][k] + T[2]*R[2][k])
                if t > ra + rb:
                    return False

            #9 cross products
            #L = A0 s B0
            ra = a[1]*abs(R[2][0]) + a[2]*abs(R[1][0])
            rb = b[1]*abs(R[0][2]) + b[2]*abs(R[0][1])
            t = abs(T[2]*R[1][0] - T[1]*R[2][0])
            if t > ra + rb:
                return False

            #L = A0 s B1
            ra = a[1]*abs(R[2][1]) + a[2]*abs(R[1][1])
            rb = b[0]*abs(R[0][2]) + b[2]*abs(R[0][0])
            t = abs(T[2]*R[1][1] - T[1]*R[2][1])
            if t > ra + rb:
                return False

            #L = A0 s B2
            ra = a[1]*abs(R[2][2]) + a[2]*abs(R[1][2])
            rb = b[0]*abs(R[0][1]) + b[1]*abs(R[0][0])
            t = abs(T[2]*R[1][2] - T[1]*R[2][2])
            if t > ra + rb:
                return False

            #L = A1 s B0
            ra = a[0]*abs(R[2][0]) + a[2]*abs(R[0][0])
            rb = b[1]*abs(R[1][2]) + b[2]*abs(R[1][1])
            t = abs( T[0]*R[2][0] - T[2]*R[0][0] )
            if t > ra + rb:
                return False

            # L = A1 s B1
            ra = a[0]*abs(R[2][1]) + a[2]*abs(R[0][1])
            rb = b[0]*abs(R[1][2]) + b[2]*abs(R[1][0])
            t = abs( T[0]*R[2][1] - T[2]*R[0][1] )
            if t > ra + rb:
                return False

            #L = A1 s B2
            ra = a[0]*abs(R[2][2]) + a[2]*abs(R[0][2])
            rb = b[0]*abs(R[1][1]) + b[1]*abs(R[1][0])
            t = abs( T[0]*R[2][2] - T[2]*R[0][2] )
            if t > ra + rb:
                return False

            #L = A2 s B0
            ra = a[0]*abs(R[1][0]) + a[1]*abs(R[0][0])
            rb = b[1]*abs(R[2][2]) + b[2]*abs(R[2][1])
            t = abs( T[1]*R[0][0] - T[0]*R[1][0] )
            if t > ra + rb:
                return False

            # L = A2 s B1
            ra = a[0]*abs(R[1][1]) + a[1]*abs(R[0][1])
            rb = b[0] *abs(R[2][2]) + b[2]*abs(R[2][0])
            t = abs( T[1]*R[0][1] - T[0]*R[1][1] )
            if t > ra + rb:
                return False

            #L = A2 s B2
            ra = a[0]*abs(R[1][2]) + a[1]*abs(R[0][2])
            rb = b[0]*abs(R[2][1]) + b[1]*abs(R[2][0])
            t = abs( T[1]*R[0][2] - T[0]*R[1][2] )
            if t > ra + rb:
                return False

            # no separating axis found,
            # the two boxes overlap 
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
       State space is by s*y*z,3 where each 3 is a point in 3D.'''
    g = {}
    Space = StateSpace(initparams.env)
    for v in Space:
        g[v] = np.inf  # this hashmap initialize all g values at inf
    return g

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

def children(initparams, x, settings = 0):
    # get the neighbor of a specific state
    allchild = []
    allcost = []
    resolution = initparams.env.resolution
    for direc in initparams.Alldirec:
        child = tuple(map(np.add, x, np.multiply(direc, resolution)))
        if any([isinobb(i, child) for i in initparams.env.OBB]):
            continue
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
    if initparams.settings == 'NonCollisionChecking':
        if dist==None:
            dist = getDist(i,j)
        collide = False
    else:
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
    # initialize Cost dictionary, could be modifed lateron
    c = defaultdict(lambda: defaultdict(dict))  # two key dicionary
    for xi in initparams.X:
        cdren = children(initparams, xi)
        for child in cdren:
            c[xi][child] = cost(initparams, xi, child)
    return c

if __name__ == "__main__":
    import time
    from env3D import R_matrix, obb
    obb1 = obb([2.6,2.5,1],[0.2,2,2],R_matrix(0,0,45))
    # obb2 = obb([1,1,0],[1,1,1],[[1/np.sqrt(3)*1,1/np.sqrt(3)*1,1/np.sqrt(3)*1],[np.sqrt(3/2)*(-1/3),np.sqrt(3/2)*2/3,np.sqrt(3/2)*(-1/3)],[np.sqrt(1/8)*(-2),0,np.sqrt(1/8)*2]])
    p0, p1 = [2.9,2.5,1],[1.9,2.5,1]
    pts = np.array([[1,2,3],[4,5,6],[7,8,9],[2,2,2],[1,1,1],[3,3,3]])
    start = time.time()
    isinbound(obb1, pts, mode='obb', factor = 0, isarray = True)
    print(time.time() - start)
    


