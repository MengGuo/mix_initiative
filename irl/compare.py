import networkx as nx
import random
import sys
import time

import numpy as np
from create_graph import construct_product

def margin_opt_path(robot_planner, opt_path, beta):
    robot_planner.reset_alpha(beta)
    robot_planner.margin_optimal(opt_path, style='ready')
    beta_best_path = list(robot_planner.run.suffix)
    return beta_best_path

def evaluate_edge_cost(robot_planner, e, beta):
    c = robot_planner.product.edge[e[0]][e[1]]['cost']
    d = robot_planner.product.edge[e[0]][e[1]]['distance']
    return c+beta*d
    
def compute_path_cost(robot_planner, path, beta):
    ac_c = 0
    ac_d = 0
    for i in range(len(path)-1):
        e = (path[i], path[i+1])
        ac_d += robot_planner.product.edge[e[0]][e[1]]['distance']
        ac_c += robot_planner.product.edge[e[0]][e[1]]['cost']
    cost = ac_c + beta*ac_d
    return [ac_c, ac_d, beta, cost]

def compute_path_d(robot_planner, path):
    ac_d = 0
    for i in range(len(path)-1):
        e = (path[i], path[i+1])
        ac_d += robot_planner.product.edge[e[0]][e[1]]['distance']
    return ac_d
        
def opt_path_match(path1, path2):
    # print 'path1', path1
    # print 'path2', path2
    score = 0
    for i,s in enumerate(path1):
        if ((i< len(path2)) and (path2[i] == s)):
            score += 1
    return score

def find_beta_via_enumeration(robot_planner, opt_path, opt_beta):
    print '------------------------------'
    print 'Find beta via enumeration starts'
    t0 = time.time()
    opt_cost = compute_path_cost(robot_planner, opt_path, opt_beta)
    beta_list = list(np.arange(0,20,0.1)) #+list(np.arange(9,10,1)) #np.arange(0, 10, 1)
    match_score = []
    cost_list = []
    prev_best_path = []
    key_beta = []
    key_path = []
    for beta in beta_list:
        robot_planner.reset_alpha(beta)
        robot_planner.optimal(style='ready')
        beta_best_path = list(robot_planner.run.suffix)
        score = opt_path_match(opt_path, beta_best_path)
        match_score.append(score)
        cost = compute_path_cost(robot_planner, beta_best_path, beta)
        cost_list.append(cost)
        if prev_best_path != beta_best_path:
            prev_best_path = list(beta_best_path)
            key_beta.append(beta)
            key_path.append(list(beta_best_path))
    index_min = max(range(len(match_score)), key= lambda j: match_score[j])
    best_beta = beta_list[index_min]
    print 'Given optimal path length: %d ||| Given beta: %.2f' %(len(opt_path), opt_beta)
    print 'In total **%d** para_dijkstra run.' %len(beta_list)
    print 'Best_beta found: %.2f ||| Given opt_beta: %.2f' %(best_beta, opt_beta)
    print 'Given optimal path cost: %s ||| Beta optimal path cost: %s ||| match score: %d ' %(str(opt_cost), str(cost_list[index_min]), match_score[index_min])
    print 'Key betas found: %s' %str(key_beta)
    print 'Find beta via enumeration done, time %.2f' %(time.time()-t0)
    return best_beta, beta_list, cost_list, match_score, key_beta, key_path


def irl(robot_planner, opt_path, opt_beta):
    print '------------------------------'
    print 'Find beta via IRL starts'
    t0 = time.time()
    opt_cost = compute_path_cost(robot_planner, opt_path, opt_beta)
    beta_list = [] 
    beta = 100.0
    beta_p = 1.0
    count = 0
    lam = 1.0
    alpha = 1.0
    cost_list = []
    match_score = []
    while (abs(beta_p-beta)>0.3):
        print 'Iteration --%d--'%count
        beta = beta_p
        opt_ac_d = compute_path_d(robot_planner, opt_path)
        marg_path = margin_opt_path(robot_planner, opt_path, beta)
        marg_ac_d = compute_path_d(robot_planner, marg_path)
        print '(opt_ac_d-marg_ac_d)', opt_ac_d-marg_ac_d
        #gradient = beta + lam*(opt_ac_d-marg_ac_d)
        gradient = lam*(opt_ac_d-marg_ac_d)
        beta_p = beta - (alpha/(count+1))*gradient
        print 'gradient:%.2f and beta_dif:%.2f' %(gradient, beta-beta_p)
        count += 1
        print 'old beta: %.2f ||| new beta: %.2f' %(beta, beta_p)
        beta_list.append(beta_p)
    print '--------------------'
    print 'In total **%d** para_dijkstra run ||| beta list: %s' %(count, str(beta_list))
    print 'Given optimal path length: %d ||| Given beta: %.2f' %(len(opt_path), opt_beta)
    print 'Best_beta found: %.2f ||| Given opt_beta: %.2f' %(beta_p, opt_beta)
    print 'Find beta via IRL done, time %.2f' %(time.time()-t0)
    print '--------------------'
    # compute score
    for beta in beta_list:
        robot_planner.reset_alpha(beta)
        robot_planner.optimal(style='ready')
        opt_beta_path = list(robot_planner.run.suffix)
        score = opt_path_match(opt_path, opt_beta_path)
        beta_opt_cost = compute_path_cost(robot_planner, opt_beta_path, beta)
        match_score.append(score)
        cost_list.append(beta_opt_cost)
        print 'Given optimal path cost: %s ||| Beta optimal path cost: %s ||| match score: %d ' %(str(opt_cost), str(beta_opt_cost), score)
    print 'beta_list: %s ||| cost_list: %s ||| match_score: %s' %(beta_list, cost_list, match_score)
    return beta_p, beta_list, cost_list, match_score


if __name__ == "__main__":
    M = 10
    N = 10
    robot_planner = construct_product(M, N)
    opt_beta = 50 #15
    robot_planner.reset_alpha(opt_beta)
    robot_planner.optimal(style='static')
    opt_path = list(robot_planner.run.suffix)
    print 'Given optimal path length: %d ||| Given beta: %.2f' %(len(opt_path), opt_beta)
    # method ONE, via enumeration
    # best_beta, beta_list, cost_list, match_score, key_beta, key_path = find_beta_via_enumeration(robot_planner, opt_path, opt_beta)
    # print 'beta_list: %s ||| match_score:%s' %(str(beta_list), str(match_score))

    # method TWO, via IRL    
    beta_p, beta_list, cost_list, match_score = irl(robot_planner, opt_path, opt_beta)
        
    


