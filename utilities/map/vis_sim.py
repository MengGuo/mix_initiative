from math import exp

import matplotlib
import matplotlib.pyplot as plt

from PIL import Image

import pickle

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
    ax.set_xlim([0,(Nx-1)*scale])
    plt.axis('off')
    plt.axis('equal')
    fig.tight_layout(pad=0)
    fig.savefig('map.pdf',bbox_inches='tight', pad_inches=0)
    return fig


def plot_traj(img_dir, A_robot_pose, A_control):
    robot_x = []
    robot_y = []
    hi_c = []
    L = range(len(A_robot_pose)-1)
    for k in L[0::5]:
        robot_pose = A_robot_pose[k]
        robot_x.append(robot_pose[1][0])
        robot_y.append(robot_pose[1][1])
        h_control = A_control[k][0]
        if (abs(h_control[0])+abs(h_control[1]))>0.1:
            hi_c.append(1)
        else:
            hi_c.append(0)
    im = Image.open(img_dir)
    pix = im.load()
    Nx = im.size[0]
    Ny = im.size[1]
    scale = 0.02
    Ns = 5
    # plot map
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for nx in range(0, Nx, Ns)+[Nx-1]:
        for ny in range(0, Ny, Ns)+[Ny-1]:
            if pix[nx,ny][0] == 0:
                ax.plot(nx*scale, (Ny-ny)*scale, color='k',
                        marker='s', markersize=1)
    # plot pre hi
    k1 = [0, 40] # initial 
    k2 = [40, 76] #hi
    k3 = [76, 90] # normal
    k4 = [95, 105] #hi
    ax.plot(robot_x[k1[0]:k1[1]], robot_y[k1[0]:k1[1]], color='b',
            linestyle='-',linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=2, zorder=2, label=r'$\tau_r^0$')
    ax.plot(robot_x[k2[0]:k2[1]], robot_y[k2[0]:k2[1]], color='r',
            linestyle='-',linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=3, zorder=5, label=r'HIL')
    ax.plot(robot_x[k3[0]:k3[1]], robot_y[k3[0]:k3[1]], color='b',
            linestyle='-',linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=3, zorder=2,)
    ax.plot(robot_x[k4[0]:k4[1]], robot_y[k4[0]:k4[1]], color='r',
            linestyle='-',linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=3, zorder=5)    
    ax.plot(robot_x[k4[1]:], robot_y[k4[1]:], color='g',
            linestyle='-',linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=3, zorder=4,label=r'$\tau_r^t$')    
    #---------- print regions of interest
    ap = ['r_0', 'r_1', 'r_2',
          'r_3', 'r_4', 'r_5',
          'r_6', 'r_7', 'r_8',
          'c_1', 'c_2', 'c_3',
          'c_4']
    roi = [(2.5, 1.5, 0.5), (8.5, 0.5, 0.5), (12.5, 1.5, 0.5),
           (17.5, 1.5, 0.5), (8.5, 4.5, 0.5), (14.5, 4.5, 0.5),
           (18.5, 4.5, 0.5), (3.0, 8.0, 0.7), (11.5, 8.5, 0.5),
           (2.0, 6.0, 1.0), (11.0, 4.0, 1.0), (17.0, 4.0, 0.7),
           (8.0, 7.0, 1.0),
             ]
    for k in range(len(roi)):
        reg = roi[k]
        rec = matplotlib.patches.Rectangle((reg[0]-reg[2], reg[1]-reg[2]), reg[2]*2, reg[2]*2, fill = True, facecolor = 'cyan', edgecolor = 'black', linewidth = 1,  alpha =0.8)
        ax.add_patch(rec)
        ax.text(reg[0], reg[1]-0.1, r'$%s$' %ap[k], fontsize = 20, fontweight = 'bold', zorder = 3)
    ax.legend(ncol=1,bbox_to_anchor=(0.8,0.6),loc='lower left', borderpad=0.1, labelspacing=0.2, columnspacing= 0.5, numpoints=3, prop={'size': 13})        
    ax.grid()
    ax.set_xlim([0,(Nx-1)*scale])
    plt.axis('off')
    plt.axis('equal')
    fig.tight_layout(pad=0)
    fig.savefig('traj_sim_1.pdf',bbox_inches='tight', pad_inches=0)
        
    
def plot_control(A_control):
    c_linear = []
    c_angular = []
    h_linear = []
    h_angular = []
    m_linear = []
    m_angular = []
    for control in A_control:
        [tele_control, navi_control, mix_control] = control
        c_linear.append(navi_control[0])
        c_angular.append(navi_control[1])
        h_linear.append(tele_control[0])
        h_angular.append(tele_control[1])
        m_linear.append(mix_control[0])
        m_angular.append(mix_control[1])
    #------------------------------ plot v
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # fig = plt.figure(figsize=(10,3))
    # ax = fig.add_subplot(111)
    # ax.plot(T, c_linear, linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[v]$')
    # ax.plot(T, h_linear, linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[v]$')
    # ax.plot(T, m_linear, linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[v]$')
    # ax.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax.grid()
    # ax.set_xlabel(r'$t(s)$')
    # ax.set_ylabel(r'$v(m/s)$')
    # ax.set_xlim(0, step*(len(A_control)))
    # ax.set_ylim(-0.5, 1.0)
    #-------------------- plot w
    step = 1.0
    T = [t*step for t in range(len(A_control))]
    fig = plt.figure(figsize=(10,3))
    ax = fig.add_subplot(111)
    ax.plot(T, c_angular, linestyle='--',
            linewidth=2.0,
            color='blue',label=r'$u_r[\omega]$')
    ax.plot(T, h_angular, linestyle='--',
            linewidth=2.0,
            color='red',label=r'$u_h[\omega]$')
    ax.plot(T, m_angular, linestyle='-',
            linewidth=2.0,
            color='black',label=r'$u[\omega]$')
    ax.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    ax.grid()
    ax.set_xlabel(r'$t(s)$')
    ax.set_ylabel(r'$\omega(rad/s)$')
    ax.set_xlim(0, step*(len(A_control)))
    ax.set_ylim(-1.1, 1.5)
    # ------------------------------ zoom in v
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # k1 = 200
    # k2 = 400
    # k3 = 600
    # fig = plt.figure(figsize=(10,3))
    # ax1 = fig.add_subplot(121)
    # ax1.plot(T[k1:k2], c_linear[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[v]$')
    # ax1.plot(T[k1:k2], h_linear[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[v]$')
    # ax1.plot(T[k1:k2], m_linear[k1:k2], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[v]$')
    # ax1.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax1.grid()
    # ax1.set_xlabel(r'$t(s)$')
    # ax1.set_ylabel(r'$v(m/s)$')
    # ax1.set_xlim(k1*step, k2*step)
    # ax1.set_ylim(-0.5, 1.0)
    # ax2 = fig.add_subplot(122)
    # ax2.plot(T[k2:k3], c_linear[k2:k3], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[v]$')
    # ax2.plot(T[k2:k3], h_linear[k2:k3], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[v]$')
    # ax2.plot(T[k2:k3], m_linear[k2:k3], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[v]$')
    # ax2.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax2.grid()
    # ax2.set_xlabel(r'$t(s)$')
    # ax2.set_ylabel(r'$v(m/s)$')
    # ax2.set_xlim(k2*step, k3*step)
    # ax2.set_ylim(-0.5, 1.0)
    # ------------------------------ zoom in w
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # k1 = 200
    # k2 = 400
    # k3 = 600
    # fig = plt.figure(figsize=(10,3))
    # ax1 = fig.add_subplot(121)
    # ax1.plot(T[k1:k2], c_angular[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[\omega]$')
    # ax1.plot(T[k1:k2], h_angular[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[\omega]$')
    # ax1.plot(T[k1:k2], m_angular[k1:k2], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[\omega]$')
    # ax1.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax1.grid()
    # ax1.set_xlabel(r'$t(s)$')
    # ax1.set_ylabel(r'$\omega(rad/s)$')
    # ax1.set_xlim(k1*step, k2*step)
    # ax1.set_ylim(-1.1, 1.5)
    # ax2 = fig.add_subplot(122)
    # ax2.plot(T[k2:k3], c_angular[k2:k3], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[\omega]$')
    # ax2.plot(T[k2:k3], h_angular[k2:k3], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[\omega]$')
    # ax2.plot(T[k2:k3], m_angular[k2:k3], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[\omega]$')
    # ax2.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax2.grid()
    # ax2.set_xlabel(r'$t(s)$')
    # ax2.set_ylabel(r'$\omega(rad/s)$')
    # ax2.set_xlim(k2*step, k3*step)
    # ax2.set_ylim(-1.1, 1.5)        
    plt.savefig(r'sim_control_w_1.pdf', bbox_inches = 'tight')

def plot_beta(A_beta):
    fig = plt.figure(figsize=(10,3))
    ax = fig.add_subplot(111)
    ax.plot(range(len(A_beta)), A_beta, color='b',
            linestyle='-', linewidth=2, marker='o', mfc='r',
            fillstyle='full', markersize=3, zorder=2)
    ax.set_xlabel(r'$Iteration$')
    ax.set_ylabel(r'$\beta_k$')
    ax.set_xlim(0, len(A_beta))
    # ax.set_ylim(0, )
    ax.grid()
    plt.savefig(r'sim_beta_1.pdf', bbox_inches = 'tight')    


if __name__ == "__main__":
    A_robot_pose, A_control, A_beta = pickle.load(open('tiago_sim_perfect.p', 'rb'))
    # draw_map('map.png')
    plot_traj('map.png', A_robot_pose, A_control)
    # plot_control(A_control)
    # plot_beta(A_beta[0])
