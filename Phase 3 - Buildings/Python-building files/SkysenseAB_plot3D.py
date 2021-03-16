from mpl_toolkits.mplot3d import Axes3D
import pylab
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import proj3d

'''
Author: Gerard Giramé Rizzo
Last edit: 10/12/2020
'''

def cuboid_data(o, size=(1, 1, 1)):
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


def plotCubeAt(pos=(0, 0, 0), size=(1, 1, 1), ax=None, **kwargs):
    # Plotting a cube element at position pos
    if ax != None:
        X, Y, Z = cuboid_data(pos, size)
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, **kwargs)


def edgecoord(pointx, pointy, pointz):
    edgex = [pointx[0], pointx[1], pointx[1], pointx[0]]
    edgey = [pointy[0], pointy[1], pointy[1], pointy[0]]
    edgez = [pointz[0], pointz[0], pointz[1], pointz[1]]
    return list(zip(edgex, edgey, edgez))


def coordConvert(x, y, lheight, uheight):
    if len(x) != len(y) and len(x) > 2:
        return
    vertices = []
    # Top layer
    vertices.append(list(zip(x, y, list(np.full(len(x), uheight)))))
    # Side layers
    for it in np.arange(len(x)):
        it1 = it + 1
        if it1 >= len(x):
            it1 = 0
        vertices.append(edgecoord([x[it], x[it1]], [y[it], y[it1]], [lheight, uheight]))
    # Bottom layer
    vertices.append(list(zip(x, y, list(np.full(len(x), lheight)))))
    return vertices


# cubic buildings
positions = [(-18, -37, 0), (50, 5, 0), (-32, -120, 0), (87, -100.5, 0), (-38, 24, 0)]
sizes = [(36, 74, 9), (25, 44, 9), (16, 40, 14), (6, 7.5, 4), (5.5, 10, 3)]
colors = ["red", "teal", "teal", "teal", "teal"]

# input values for any polygon buildings
x = [96, 96, 159, 159, 165, 165, 110, 110, 144, 144, 96]
y = [20, 43, 43, 1, 1, -51, -51, -33, -33, 20, 20]
z = [0, 12]
vec = coordConvert(x, y, z[0], z[1])

x1 = [-16, 59, 59, 72, 72, 57, 57, -9, -9, -16, -16]
y1 = [-100, -100, -116, -116, -140, -140, -188, -188, -130, -130, -100]
z1 = [0, 9]
vec1 = coordConvert(x1, y1, z1[0], z1[1])

x2 = [126, 154, 154, 163, 163, 153, 153, 126, 126]
y2 = [-90, -90, -83, -83, -123, -123, -139, -139, -90]
z2 = [0, 8]
vec2 = coordConvert(x2, y2, z2[0], z2[1])

x3 = [-59, -59, -91, -91, -69, -59]
y3 = [-80, -162, -162, -109, -80, -80]
z3 = [0, 15]
vec3 = coordConvert(x3, y3, z3[0], z3[1])

x4 = [-13, -13, -1, -1, -13, -13, 4, 4, 62, 62, 4, 4, -13]
y4 = [177, 193, 193, 218, 218, 237, 237, 243, 243, 171, 171, 177, 177]
z4 = [0, 26]
vec4 = coordConvert(x4, y4, z4[0], z4[1])

x5 = [-44, -44, -69, -69, -93, -93, -69, -69, -111, -111, -44]
y5 = [35, -31, -31, 12, 12, -9, -9, -31, -31, 35, 35]
z5 = [0, 17]
vec5 = coordConvert(x5, y5, z5[0], z5[1])

fig = plt.figure(num=None, figsize=(10, 8), dpi=80, facecolor='w', edgecolor='k')
ax = fig.gca(projection='3d')

for p, s, c in zip(positions, sizes, colors):
    plotCubeAt(pos=p, size=s, ax=ax, color=c, alpha=.25, edgecolor='k')

plt.gca().add_collection3d(Poly3DCollection(vec, alpha=.5, edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec1, alpha=.5, edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec2, alpha=.5, edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec3, alpha=.5, edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec4, alpha=.5, edgecolor='k', facecolor='teal'))
plt.gca().add_collection3d(Poly3DCollection(vec5, alpha=.5, edgecolor='k', facecolor='teal'))

ax.set_xlim([-120, 170])
ax.set_ylim([-200, 250])
ax.set_zlim([0, 80])
# Writing The Title of The Plot and axis
ax.set_title(r'$3D\; Visualization\; -\; Skysense\; AB$', fontsize=15)
ax.set_xlabel('$x\; [m]$', color='b')
ax.set_ylabel('$y\; [m]$', color='b')
ax.set_zlabel('$z\; [m]$', color='b')

# Stablishing the plots of our legend labels
gray_proxy = plt.Rectangle((0, 0), 1, 1, fc='teal', alpha=.75, edgecolor='k')
red_proxy = plt.Rectangle((0, 0), 1, 1, fc='red', alpha=.75, edgecolor='k')
k_dot = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='k')
blue_dot = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='blue')

# Drawing Our Legend
ax.legend([gray_proxy, red_proxy, k_dot, blue_dot],
          [r'$Buildings$', r'$Skysense\; AB\; -\; RTH$', r'$AP\; (0,0,11)$', r'$Drone\; path$'], numpoints=1,
          loc='upper left')
#ax.view_init(azim=0, elev=90)

# drone path
drone_x = [50, 0, -50, 0]
drone_y = [0, 50, 0, -50]
drone_z = [50, 50, 50, 50]

drone_SNR = [30, 25, 10, 20]
drone_thrgh = [64, 34, 39, 50]

ax.scatter(0, 0, 11, c='black')  # AP (0,0,11)
#ax.scatter(drone_x, drone_y, drone_z, c='b')  # drone (x,y,z)

for k in range(len(drone_x)):
    scatter_node = ax.scatter(drone_x[k], drone_y[k], drone_z[k], c='b')  # drone (x,y,z)

    x2, y2, _ = proj3d.proj_transform(drone_x[k],drone_y[k],drone_z[k], ax.get_proj())

    label = pylab.annotate(
        r'$SNR=%.2f$ dB' '\n' '$thrgh=%.2f $ Mbps' % (drone_SNR[k],drone_thrgh[k],),
        xy = (x2, y2), xytext = (-20, 20),
        textcoords = 'offset points', ha = 'right', va = 'bottom',
        bbox = dict(boxstyle = 'round,pad=0.5', fc = 'wheat', alpha = 0.5),
        arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))

    plt.pause(2)
    scatter_node.set_visible(False)
    label.remove()

pylab.show()

#plt.show()


