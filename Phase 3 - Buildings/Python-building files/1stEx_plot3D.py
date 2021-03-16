from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

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

positions = [(20,-20,0),(-20,-20,0)]
sizes = [(10,40,15), (10,40,15)]
colors = ["gray","gray"]

#drone path
drone_x = [50, 0, -50, 0]
drone_y = [0, 30, 0, -30]
drone_z = [25, 25, 25, 25]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter(0, 0, 0, c='red') #AP (0,0,0)
ax.scatter(drone_x, drone_y, drone_z, c='b') #drone (x,y,z)

for p,s,c in zip(positions,sizes,colors):
    plotCubeAt(pos=p, size=s, ax=ax, color=c)

ax.set_xlim([-50,50])
ax.set_ylim([-30,30])
ax.set_zlim([0,50])
# Writing The Title of The Plot
ax.set_title(r'$3D\; Visualization\; -\; Simplistic$', fontsize=10)

# Stablishing the plots of our legend labels
gray_proxy = plt.Rectangle((0, 0), 1, 1, fc='gray')
red_proxy = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='red')
blue_proxy = plt.Line2D([0], [0], linestyle="none", marker='o', markersize=10, color='blue')

# Drawing Our Legend
ax.legend([gray_proxy, red_proxy, blue_proxy], [r'$Buildings$', r'$AP\; (0,0,0)$', r'$Drone\; path$'], numpoints=1, loc='upper left')

plt.show()