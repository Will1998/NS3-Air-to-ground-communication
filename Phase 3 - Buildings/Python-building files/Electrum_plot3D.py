import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

'''
Author: Gerard Giramé Rizzo
Last edit: 1/12/2020
'''

def cuboid_data(o, size=(1,1,1)):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],
         [o[1], o[1], o[1], o[1], o[1]],
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]
    z = [[o[2], o[2], o[2], o[2], o[2]],
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]
    return np.array(x), np.array(y), np.array(z)

def plotCubeAt(pos=(0,0,0), size=(1,1,1), ax=None,**kwargs):
    # Plotting a cube element at position pos
    if ax !=None:
        X, Y, Z = cuboid_data( pos, size )
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, **kwargs)

def edgecoord(pointx,pointy,pointz):
    edgex=[pointx[0],pointx[1],pointx[1],pointx[0]]
    edgey=[pointy[0],pointy[1],pointy[1],pointy[0]]
    edgez=[pointz[0],pointz[0],pointz[1],pointz[1]]
    return list(zip(edgex,edgey,edgez))

def coordConvert(x,y,lheight,uheight):
    if len(x) != len(y) and len(x)>2:
        return
    vertices=[]
    #Top layer
    vertices.append(list(zip(x,y,list(np.full(len(x),uheight)))))
    # Side layers
    for it in np.arange(len(x)):
        it1=it+1
        if it1>=len(x):
            it1=0
        vertices.append(edgecoord([x[it],x[it1]],[y[it],y[it1]],[lheight,uheight]))
    #Bottom layer
    vertices.append(list(zip(x,y,list(np.full(len(x),lheight)))))
    #print(np.array(vertices))
    return vertices

#cubic buildings
positions = [(-72,0,0)]
sizes = [(12,9,5)]
colors = ["teal"]

#input values for any polygon buildings
x=[9, 70, 70, 51, 51, 33, 33, 76, 76, 61, 61, 38, 38, 86, 86, 67, 67, -5, -5, 9, 9]
y=[3, 3, 17, 17, 23, 23, 37, 37, 50, 50, 57, 57, 73, 73, 83, 83, 90, 90, 83, 83, 3]
z=[0,21]
vec=coordConvert(x,y,z[0],z[1])

x1=[-17,-17, -86, -86, -75, -75, -86, -86, -17]
y1=[14, 59, 59, 46, 46, 24, 24, 14, 14]
z1=[0,24]
vec1=coordConvert(x1,y1,z1[0],z1[1])

x2=[-5, -5, -89, -89, -19, -19, -5]
y2=[82, 90, 90, 70, 70, 82, 82]
z2=[0,22]
vec2=coordConvert(x2,y2,z2[0],z2[1])

x3=[-23, -14,  -14, -36, -36, -43, -43,-84, -84, -43, -43, -15, -15,  -4,  -4,   6,   6,   38,  38,  52,  52,
     41,  41,  52,  52,  38,  38, 17,  17,  45,  45,  69,  69,  74,  74,  48,  48,  -42, -42, -83, -83,
    -78, -78, -56, -56, -23 ]
y3=[-102, -88 ,-72,  -72, -64, -64, -72,-72, -12, -12, -19, -19, -12, -12, -16, -16, -12,  -12, -19, -19, -30,
    -30, -54, -54, -65, -65, -72, -72,-102,-102,-95, -95, -102,-102,-112,-112,-120, -120,-112,-112,-102,
    -102,-95, -95, -102,-102]
z3=[0,26]
vec3=coordConvert(x3,y3,z3[0],z3[1])


#drone path
drone_x = [50, 0, -50, 0]
drone_y = [0, 50, 0, -50]
drone_z = [50, 50, 50, 50]

fig = plt.figure(num=None, figsize=(12, 10), dpi=80, facecolor='w', edgecolor='k')
ax = fig.gca(projection='3d')

for p,s,c in zip(positions,sizes,colors):
    plotCubeAt(pos=p, size=s, ax=ax, color=c, alpha=.25, edgecolor='k')



#plt.subplot(111,projection='3d')
plt.gca().add_collection3d(Poly3DCollection(vec, alpha=.25,edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec1, alpha=.25,edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec2, alpha=.25,edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec3, alpha=.25,edgecolor='k', facecolor='red'))

ax.scatter(0, 0, 1.7, c='black') #AP (0,0,0)
ax.scatter(drone_x, drone_y, drone_z, c='b') #drone (x,y,z)

ax.set_xlim([-115,115])
ax.set_ylim([-130,115])
ax.set_zlim([0,80])
# Writing The Title of The Plot and axis
ax.set_title(r'$3D\; Visualization\; -\; Electrum$', fontsize=15)
ax.set_xlabel('$x\; [m]$', color='b')
ax.set_ylabel('$y\; [m]$', color='b')
ax.set_zlabel('$z\; [m]$', color='b')

# Stablishing the plots of our legend labels
gray_proxy = plt.Rectangle((0, 0), 1, 1, fc='teal', alpha=.75, edgecolor='k')
red_proxy = plt.Rectangle((0, 0), 1, 1, fc='red', alpha=.75, edgecolor='k')
red_dot = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='k')
blue_dot = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='blue')

# Drawing Our Legend
ax.legend([gray_proxy, red_proxy,  red_dot, blue_dot], [r'$Buildings$', r'$Electrum; -\; RTH$', r'$AP\; (0,0,1.5)$', r'$Drone\; path$'], numpoints=1, loc='upper left')
#ax.view_init(azim=0, elev=90)
plt.show()