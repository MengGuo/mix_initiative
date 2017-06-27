#!/usr/bin/python
# -*- coding: utf-8 -*-

# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG

import time
from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner
from itertools import product
import random
from math import floor



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
                g.add_edge(nodef, nodet, weight=random.random()*unit_cost*(abs(dx)+abs(dy)))
    return g


def construct_product(N):
    # create motion model
    robot_motion = create_rectworld(N,N,True)
    robot_motion.set_initial((0,0))
    # empty action model
    robot_action = ActionModel(dict())
    # complete robot model
    robot_model = MotActModel(robot_motion, robot_action)
    # task formula
    hard_task = '[]! x%dy%d' %(floor(0.5*N), floor(0.5*N))
    soft_task = '([]<> x%dy%d) && ([]<> x%dy%d) && ([]<> x%dy%d)' %(N-1,0,N-1,N-1,0,N-1)
    print '------------------------------'
    print 'hard_task: %s ||| soft_task: %s' %(str(hard_task), str(soft_task))
    print '------------------------------'
    # set planner
    alpha = 10
    robot_planner = ltl_planner(robot_model, hard_task, soft_task, alpha)
    return robot_planner
