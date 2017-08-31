from math import exp

import matplotlib
import matplotlib.pyplot as plt

import numpy
from PIL import Image

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def draw_map(img_dir):
    im = Image.open(img_dir)
    pix = im.load()
    Nx = im.size[0]
    Ny = im.size[1]
    scale = 0.02
    # print im.size
    # x = 100
    # y= 100
    # print pix[x,y] #Get the RGBA Value of the a pixel of an image
    # print pix[0,0]
    # (1000, 450)
    # (255, 255, 255)
    # (0, 0, 0)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for nx in range(0, Nx, 5)+[Nx-1]:
        for ny in range(0, Ny, 5)+[Ny-1]:
            if pix[nx,ny][0] == 0:
                ax.plot(nx*scale, (Ny-ny)*scale, color='k',
                        marker='s', markersize=1)
    ax.set_xlim([0,Nx-1])
    plt.axis('off')
    plt.axis('equal')
    fig.tight_layout(pad=0)
    fig.savefig('map.pdf',bbox_inches='tight', pad_inches=0)
    return fig


if __name__ == "__main__":
    draw_map('map.png')
