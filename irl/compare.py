import networkx as nx
import random
import sys
import time
from itertools import product
import scipy.stats
from math import sqrt

import numpy as np


def create_rectworld(Xs, Ys, eight_connected=False):
    '''
    create a rectangular world with square cells.
    eight_connected True if diagonal motion is allowed.
    '''
    X_range = range(0, Xs)
    Y_range = range(0, Ys)
    G = nx.DiGraph()
    for X in X_range:
        for Y in Y_range:
            G.add_node((X,Y))
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
                G.add_edge(nodef, nodet)
    return G

def c_gaussian(N, e, mag_c=10):
    mean = (0.7*N, 0.3*N)
    std = 0.15*N
    dist = sqrt((e[1][0]-mean[0])**2+(e[1][1]-mean[1])**2)
    c  = mag_c*scipy.stats.norm(0, std).pdf(dist)
    return c

def d_gaussian(N, e, mag_d=10):
    mean = (0.4*N, 0.6*N)
    std = 0.15*N
    dist = sqrt((e[1][0]-mean[0])**2+(e[1][1]-mean[1])**2)
    d  = mag_d*scipy.stats.norm(0, std).pdf(dist)    
    return d


def create_graph(N=8):
    print '------------------------------'
    G = create_rectworld(N, N)
    for e in G.edges():
        G.edge[e[0]][e[1]]['cost'] = c_gaussian(N, e)
        G.edge[e[0]][e[1]]['distance'] = d_gaussian(N, e)
    G.graph['start'] = (0, 0)
    G.graph['goal'] = (N-1, N-1)
    print 'DiGraph created: %d nodes, %d edges' %(len(G.nodes()),len(G.edges()))
    print '------------------------------'
    return G

def param_dijkstra(G, beta):
    start = G.graph['start']
    goal = G.graph['goal']
    G_beta = nx.DiGraph()
    for e in G.edges():
        G_beta.add_edge(e[0],e[1],
                        weight=evaluate_edge_cost(G,e,beta))
    path = nx.shortest_path(G_beta, start, goal, 'weight')
    return path

def margin_opt_path(G, opt_path, beta):
    start = G.graph['start']
    goal = G.graph['goal']
    path_edges = zip(opt_path[0::2], opt_path[1::2])
    G_beta = nx.DiGraph()
    for e in G.edges():
        if e in path_edges:
            k = 1
        else:
            k = 0
        w = evaluate_edge_cost(G,e,beta)-k
        G_beta.add_edge(e[0], e[1], weight=w)
    path = nx.shortest_path(G_beta, start, goal)
    return path    

def evaluate_edge_cost(G, e, beta):
    c = G.edge[e[0]][e[1]]['cost']
    d = G.edge[e[0]][e[1]]['distance']
    return c+beta*d
    
def compute_path_cost(G, path, beta):
    ac_c = 0
    ac_d = 0
    for i in range(len(path)-1):
        e = (path[i], path[i+1])
        ac_d += G.edge[e[0]][e[1]]['distance']
        ac_c += G.edge[e[0]][e[1]]['cost']
    cost = ac_c + beta*ac_d
    return [ac_c, ac_d, beta, cost]

def compute_path_d(G, path):
    ac_d = 0
    for i in range(len(path)-1):
        e = (path[i], path[i+1])
        ac_d += G.edge[e[0]][e[1]]['distance']
    return ac_d
        
def opt_path_match(path1, path2):
    # print 'path1', path1
    # print 'path2', path2
    score = 0
    for i,s in enumerate(path1):
        if path2[i] == s:
            score += 1
    return score

def find_beta_via_enumeration(G, opt_path, opt_beta):
    print '------------------------------'
    print 'Find beta via enumeration starts'
    t0 = time.time()
    opt_cost = compute_path_cost(G, opt_path, opt_beta)
    beta_list = range(0,8)
    match_score = []
    cost_list = []
    for beta in beta_list:
        beta_best_path = param_dijkstra(G, beta)
        score = opt_path_match(opt_path, beta_best_path)
        match_score.append(score)
        cost = compute_path_cost(G, beta_best_path, beta)
        cost_list.append(cost)
    index_min = max(range(len(match_score)), key= lambda j: match_score[j])
    best_beta = beta_list[index_min]
    print 'In total **%d** para_dijkstra run.' %len(beta_list)
    print 'Best_beta found: %.2f ||| Given opt_beta: %.2f' %(best_beta, opt_beta)
    print 'Given optimal path cost: %s ||| Beta optimal path cost: %s ||| match score: %d ' %(str(opt_cost), str(cost_list[index_min]), match_score[index_min])
    print 'Find beta via enumeration done, time %.2f' %(time.time()-t0)
    return best_beta, beta_list, cost_list, match_score

def irl(G, opt_path, opt_beta):
    print '------------------------------'
    print 'Find beta via IRL starts'
    t0 = time.time()
    opt_cost = compute_path_cost(G, opt_path, opt_beta)
    beta_list = []
    beta = 100
    beta_p = 1
    count = 0
    lam = 1
    alpha = 1
    while (abs(beta_p-beta)>0.3):
        print 'Iteration --%d--'%count
        beta = beta_p
        opt_ac_d = compute_path_d(G, opt_path)
        marg_path = margin_opt_path(G, opt_path, beta)
        marg_ac_d = compute_path_d(G, marg_path)
        gradient = beta + lam*(opt_ac_d-marg_ac_d)
        beta_p = beta - (alpha/(count+1))*gradient
        count += 1
        print 'old beta: %.2f ||| new beta: %.2f' %(beta, beta_p)
        beta_list.append(beta_p)
    print 'In total **%d** para_dijkstra run ||| beta list: %s' %(count, str(beta_list))
    print 'Best_beta found: %.2f ||| Given opt_beta: %.2f' %(beta_p, opt_beta)
    print 'Find beta via IRL done, time %.2f' %(time.time()-t0)
    return beta_p, beta_list


N = 50
G = create_graph(N)
opt_beta = 10
opt_path = param_dijkstra(G, opt_beta)
print 'Given optimal path length: %d ||| Given beta: %.2f' %(len(opt_path), opt_beta)
# method ONE, via enumeration

best_beta, beta_list, cost_list, match_score = find_beta_via_enumeration(G, opt_path, opt_beta)
print 'beta_list: %s ||| match_score:%s' %(str(beta_list), str(match_score))

# method TWO, via IRL    
# beta_p, beta_list = irl(G, opt_path, opt_beta)
# match_score = []
# for beta in beta_list:
#     beta_best_path = param_dijkstra(G, beta)
#     score = opt_path_match(opt_path, beta_best_path)
#     match_score.append(score)
# print 'match score', match_score
        
    


