from math import exp

import matplotlib
import matplotlib.pyplot as plt

import numpy


matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def rho(s):
    if (s > 0):
        return exp(-1.0/s)
    else:
        return 0
        
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 0.4
    epsilon = 0.1
    mix_control = 0
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control = navi_control + gain*tele_control
    #mix_control = (1-gain)*navi_control + gain*tele_control
    return mix_control, gain

def draw_mix_u(d, navi_u, tele_u, mix_u, gain):
    fig = plt.figure(figsize=(10,4))
    ax = fig.add_subplot(111)
    ax.plot(d, navi_u, color='r', linestyle='-',
            linewidth=3, marker='o', mfc='r',
            fillstyle='full', markersize=6,label=r'$u_r$')
    ax.plot(d, tele_u,color='g', linestyle='-',
            linewidth=3, marker='d', mfc='g',
            fillstyle='full', markersize=6,label=r'$u_h$')
    ax.plot(d, mix_u, color='b', linestyle='-',
            linewidth=3, marker='>', mfc='b',
            fillstyle='full', markersize=6,label=r'$u$')
    ax.plot(d, gain, color='k', linestyle='-',
            linewidth=3, marker='*', mfc='k',
            fillstyle='full', markersize=6,label=r'$\kappa$')
    ax.set_xlabel(r'$d_t(m)$',fontsize=20)
    ax.legend(loc = (.6,.62), labelspacing=0.7, numpoints=3, handlelength=2.5, ncol=2, prop={'size':20})
    ax.set_xlim(0, d[-1]+0.01)
    fig.tight_layout()
    plt.grid()
    plt.savefig('mix_u.pdf',bbox_inches='tight')
    return fig


if __name__ == "__main__":
    #D = [k*0.05 for k in range(30)]
    D = list(numpy.arange(0,0.4,0.05)) + list(numpy.arange(0.4,0.5,0.01)) + list(numpy.arange(0.5,1.1,0.05))
    navi_U = [2 for k in range(len(D))]
    tele_U = [1.5 for k in range(len(D))]
    mix_U = []
    Gain = []
    for k in range(len(D)):
        d = D[k]
        navi_u = navi_U[k]
        tele_u = tele_U[k]
        mix_u, gain = smooth_mix(tele_u, navi_u, d)
        mix_U.append(mix_u)
        Gain.append(gain)
    draw_mix_u(D, navi_U, tele_U, mix_U, Gain)
