import numpy as np
from numpy.matlib import repmat
import pyrr as pyrr
# plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits.mplot3d as plt3d

def getRay(x,y):
    direc = [y[0]-x[0],y[1]-x[1],y[2]-x[2]]
    return np.array([x,direc])

def getAABB(blocks):
    AABB = []
    for i in blocks:
        AABB.append(np.array([np.add(i[0:3],-0),np.add(i[3:6],0)])) # make AABBs alittle bit of larger
    return AABB

def getDist(pos1,pos2):
    return np.sqrt(sum([(pos1[0]-pos2[0])**2,(pos1[1]-pos2[1])**2,(pos1[2]-pos2[2])**2]))

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  #clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  #fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    #fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.15, linewidths=1, edgecolors='k')
    #pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h

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
    x = np.random.uniform(initparams.env.boundary[0:3],initparams.env.boundary[3:6])
    if isinside(initparams,x):
        return sampleFree(initparams)
    else: return np.array(x)

def isinside(initparams,x):
    '''see if inside obstacle'''
    for i in initparams.env.blocks:
        if i[0] <= x[0] < i[3] and i[1] <= x[1] < i[4] and i[2] <= x[2] < i[5]:
            return True
    return False

def isCollide(initparams,x,y):
    '''see if line intersects obstacle'''
    ray = getRay(x,y)
    dist = getDist(x,y)
    for i in getAABB(initparams.env.blocks):
        shot = pyrr.geometric_tests.ray_intersect_aabb(ray,i)
        if shot is not None:
            dist_wall = getDist(x,shot)
            if dist_wall <= dist: # collide
                return True
    return False

def nearest(initparams,x):
    V = np.array(initparams.V)
    if initparams.i == 0:
        return initparams.V[0]
    xr = repmat(x,len(V),1)
    dists = np.linalg.norm(xr - V,axis = 1)
    return initparams.V[np.argmin(dists)]

def steer(initparams,x,y):
    direc = (y - x)/np.linalg.norm(y - x)
    xnew = x + initparams.stepsize*direc
    return xnew

def near(initparams,x,r=2):
    #TODO: r = min{gamma*log(card(V)/card(V)1/d),eta}
    V = np.array(initparams.V)
    if initparams.i == 0:
        return initparams.V[0]
    xr = repmat(x,len(V),1)
    inside = np.linalg.norm(xr - V,axis = 1) < r
    nearpoints = V[inside]
    return np.array(nearpoints)

def cost(initparams,x):
    '''here use the additive recursive cost function'''
    if all(x == initparams.env.start):
        return 0
    xparent = initparams.Parent[str(x[0])][str(x[1])][str(x[2])]
    return cost(initparams,xparent) + getDist(x,xparent)

def visualization(initparams):
    V = np.array(initparams.V)
    E = np.array(initparams.E)
    Path = np.array(initparams.Path)
    start = initparams.env.start
    goal = initparams.env.goal
    ax = plt.subplot(111,projection='3d')
    ax.view_init(elev=0., azim=90)
    ax.clear()
    draw_block_list(ax,initparams.env.blocks)
    if E != []:
        for i in E:
            xs = i[0][0],i[1][0]
            ys = i[0][1],i[1][1]
            zs = i[0][2],i[1][2]
            line = plt3d.art3d.Line3D(xs, ys, zs)
            ax.add_line(line)

    if Path != []:
        for i in Path:
            xs = i[0][0],i[1][0]
            ys = i[0][1],i[1][1]
            zs = i[0][2],i[1][2]
            line = plt3d.art3d.Line3D(xs, ys, zs,color='r')
            ax.add_line(line)
    
    ax.plot(start[0:1],start[1:2],start[2:],'go',markersize=7,markeredgecolor='k')
    ax.plot(goal[0:1],goal[1:2],goal[2:],'ro',markersize=7,markeredgecolor='k')
    ax.scatter3D(V[:,0], V[:,1], V[:,2])
    plt.xlim(initparams.env.boundary[0],initparams.env.boundary[3])
    plt.ylim(initparams.env.boundary[1],initparams.env.boundary[4])
    ax.set_zlim(initparams.env.boundary[2],initparams.env.boundary[5])
    plt.xlabel('x')
    plt.ylabel('y')
    if not Path != []:
        plt.pause(0.001)
    else: plt.show()

def path(initparams,Path=[],dist=0):
    x = initparams.env.goal
    while not all(x==initparams.env.start):
        x2 = initparams.Parent[str(x[0])][str(x[1])][str(x[2])]
        Path.append(np.array([x,x2]))
        dist += getDist(x,x2)
        x = x2
    return Path,dist


