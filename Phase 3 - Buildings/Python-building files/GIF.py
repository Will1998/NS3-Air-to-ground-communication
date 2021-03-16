import imageio
import glob

'''
Author: Gerard Giramé Rizzo
Last edit: 01/12/2020
'''

images = []

for filename in sorted(glob.glob('C:/Users/usuari/Desktop/KTH/Semestre 3/CSD/NS3 - Project/Building/Skysense results/2Dheatmap_thrgh_Z_*_Noaggr.png')):
    images.append(imageio.imread(filename))
imageio.mimsave('C:/Users/usuari/Desktop/KTH/Semestre 3/CSD/NS3 - Project/Building/Skysense results/thrgh_Noagrr_gif.gif', images, duration=0.40)