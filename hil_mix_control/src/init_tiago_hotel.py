from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


import time


##############################
# motion FTS 
ap = ['r0', 'r1', 'r2',
      'r3', 'r4', 'r5',
      'r6', 'r7', 'r8',
      'c1', 'c2', 'c3',
      'c4']
loc = [(2.5, 1.5, 0.5), (8.5, 0.5, 0.5), (12.5, 1.5, 0.5),
       (17.5, 1.5, 0.5), (8.5, 4.5, 0.5), (14.5, 4.5, 0.5),
       (18.5, 4.5, 0.5), (3.5, 8.5, 0.5), (11.5, 8.5, 0.5),
       (1.0, 5.0, 1.0), (11.0, 4.0, 1.0), (17.0, 4.0, 1.0),
       (8.0, 7.0, 1.0),
         ]
regions = dict()
for k in range(len(ap)):
    regions[loc[k]] = set([ap[k],])
init_pose = loc[0]
robot_motion = MotionFts(regions, set(ap), 'hotel' )
robot_motion.set_initial(list(init_pose))
edges = [(0,1), (0,2), (0,4), (0,9), 
         (1,4), (1,2), 
         (2,4), 
         (3,11),
         (5,8), (5,11), (5,12),
         (6,11),
         (7,9),(7,12),
         (8,12),(8,11),
         (9,12),
         (11,12)]
edge_list = [(loc[e[0]], loc[e[1]]) for e in edges]
robot_motion.add_un_edges(edge_list, unit_cost = 2)


##############################
# action FTS
############# no action model
action = dict()

robot_action = ActionModel(action)

robot_model = [robot_motion, init_pose, robot_action]
##############################
# complete robot model
robot_full_model = MotActModel(robot_motion, robot_action)

##############################
#specify soft and hard tasks

# case one 
# hard_task = '(([]<> (r0 && <> (r8 && <> r7))) && ([]<> (r2 && <> r6)) && ([] !r5))'
# soft_task = '([]! c4)'

# case two
hard_task = '(([]<> r2) && ([]<> r3) && ([]<> r8))'
soft_task = '(([]<> r4) && ([]<> r6))'

sys_model = [robot_full_model, hard_task, soft_task]

