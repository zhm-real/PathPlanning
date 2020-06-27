import numpy as np

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

def StateSpace(boundary,factor=0):
    '''This function is used to get nodes and discretize the space.
       State space is by x*y*z,3 where each 3 is a point in 3D.'''
    xmin,xmax = boundary[0]+factor,boundary[3]-factor
    ymin,ymax = boundary[1]+factor,boundary[4]-factor
    zmin,zmax = boundary[2]+factor,boundary[5]-factor
    xarr = np.arange(xmin,xmax,1)
    yarr = np.arange(ymin,ymax,1)
    zarr = np.arange(zmin,zmax,1)
    V = np.meshgrid(xarr,yarr,zarr)
    VV = np.reshape(V,[3,len(xarr)*len(yarr)*len(zarr)]) # all points in 3D
    Space = {}
    for v in VV.T:
        Space[hash3D(v)] = 0 # this hashmap initialize all g values at 0
    return Space

if __name__ == "__main__":
    from env3D import env
    env = env(resolution=1)
    Space = StateSpace(env.boundary,0)
    t = np.array([3.0,4.0,5.0])
    h = Heuristic(Space,t)
    print(h[hash3D(t)])