#!/usr/bin/python
# -*- coding: utf-8 -*-

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG

import time
from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner
from itertools import product
import random
from math import floor


import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon

import pickle


matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def create_rectworld(Xs, Ys, eight_connected=False):
    '''
    create a rectangular world with square cells.
    eight_connected True if diagonal motion is allowed.
    '''
    node_dict = {}
    symbols = []
    rectworld_nodename = "x{:d}y{:d}".format
    X_range = range(0, Xs)
    Y_range = range(0, Ys)
    for X in X_range:
        for Y in Y_range:
            node_name = rectworld_nodename(X, Y)
            node_dict[(X,Y)] = set([node_name,])
            symbols.append(node_name)
    g = MotionFts(node_dict, symbols, 'rectworld')
    for X in X_range:
        for Y in Y_range:
            nodef = (X, Y)
            if eight_connected:
                offsets = product([-1,0,1],[-1,0,1])
            else:
                offsets = [(0,0),(0,1),(0,-1),(1,0),(-1,0)]
            for (dx,dy) in offsets:
                xt = X + dx
                yt = Y + dy
                if xt not in X_range or yt not in Y_range:
                    continue
                nodet = (xt, yt)
                unit_cost = 1
                if (abs(xt) + abs(yt) != 0):
                    g.add_edge(nodef, nodet, weight=unit_cost*(abs(dx)+abs(dy)+random.random()))
                else:
                    g.add_edge(nodef, nodet, weight=unit_cost*0.000001)
    return g


def construct_product(M,N):
    # create motion model
    robot_motion = create_rectworld(M,N,True)
    draw_world(robot_motion, M, N)
    robot_motion.set_initial((0,0))
    # empty action model
    robot_action = ActionModel(dict())
    # complete robot model
    robot_model = MotActModel(robot_motion, robot_action)
    # task formula
    hard_task = '[]! x%dy%d' %(floor(0.5*M), floor(0.5*N))
    soft_task = '([]<> x%dy%d) && ([]<> x%dy%d) && ([]<> x%dy%d)' %(M-1,0,M-1,N-1,0,N-1)
    print '------------------------------'
    print 'hard_task: %s ||| soft_task: %s' %(str(hard_task), str(soft_task))
    print '------------------------------'
    # set planner
    alpha = 10
    robot_planner = ltl_planner(robot_model, hard_task, soft_task, alpha)
    return robot_planner


def draw_beta(enum_beta):
    [best_beta, beta_list, cost_list, match_score, key_beta, key_path] = enum_beta
    K = range(len(beta_list))
    K_ac_c = [cost_list[k][0] for k in K]
    K_ac_d = [cost_list[k][1] for k in K]    
    #[ac_c, ac_d, beta, cost]
    fig, ax1 = plt.subplots()
    ax1.plot(beta_list, K_ac_c, 'b-',linewidth=5,label=r'$\mathbf{\alpha}_1$')
    ax1.set_xlabel(r'\beta',fontsize=20)
    # Make the y-axis label, ticks and tick labels match the line color.
    ax1.set_ylabel(r'$\mathbf{\alpha}_1$',fontsize=20)
    legend1 = ax1.legend(loc = (.75,.5), labelspacing=1.1, numpoints=2, handlelength=2, prop={'size':15})    
    ax2 = ax1.twinx()
    ax2.plot(beta_list, K_ac_d, 'r--',linewidth=5, label=r'$\mathbf{\alpha}_3$')
    ax2.set_ylabel(r'$\mathbf{\alpha}_3$',fontsize=20)
    ax2.legend(loc = (.75, .62), frameon = True, labelspacing=1.1, numpoints=2, handlelength=2, prop={'size':15})
    fig.tight_layout()
    plt.grid()
    plt.savefig('figures/enum_beta.pdf',bbox_inches='tight')
    print 'key_beta', key_beta
    print 'K_ac_c', set(K_ac_c)
    print 'K_ac_d', set(K_ac_d)
    return fig
    
def draw_world(robot_motion, M, N):
    figure = plt.figure()
    ax = figure.add_subplot(1,1,1)
    WS_d = 0.5
    #----- draw the workspace
    bases = set([(M-1,0),(M-1,N-1),(0,N-1)])
    for node in robot_motion.nodes():
        if node in bases:
            color = 'yellow'
        elif node == (floor(0.5*M), floor(0.5*N)):
            color = 'red'
        elif node == ((0,0)):
            color = 'green'
        else:
            color = 'white'
        rec = matplotlib.patches.Rectangle((node[0], node[1]),
                                               WS_d*2, WS_d*2,
                                               fill = True,
                                               facecolor = color,
                                               edgecolor = 'black',
                                               linewidth = 1,
                                               ls = ('solid'),)
        ax.add_patch(rec)
    ax.set_aspect('equal')
    ax.set_xlim(0, M)
    ax.set_ylim(0, N)
    ax.set_xlabel(r'$x(m)$')
    ax.set_ylabel(r'$y(m)$')    
    plt.savefig('figures/grid.pdf',bbox_inches='tight')
    return figure    


if __name__ == "__main__":
    # M = 10
    # N = 10
    # robot_planner = construct_product(M, N)
    enum_beta = pickle.load(open('figures/enum_beta.p','rb'))
    draw_beta(enum_beta)
    
