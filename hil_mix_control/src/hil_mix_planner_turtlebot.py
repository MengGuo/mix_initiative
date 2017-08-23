#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('hil_mix_control')
import rospy
import sys

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from hil_mix_control.msg import task



from math import pi as PI
from math import atan2, sin, cos, sqrt, exp

from tf.transformations import euler_from_quaternion, quaternion_from_euler


from init_sys import sys_model
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner

import pickle


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

def rho(s):
    if (s > 0):
        return exp(-1.0/s)
    else:
        return 0
        
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 0.3
    epsilon = 0.1
    mix_control = [0, 0]
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    # mix_control[0] = navi_control[0] + gain*tele_control[0]
    # mix_control[1] = navi_control[1] + gain*tele_control[1]
    mix_control[0] = (1-gain)*navi_control[0] + gain*tele_control[0]
    mix_control[1] = (1-gain)*navi_control[1] + gain*tele_control[1]
    return mix_control, gain

def TaskCallback(taskdata):
    # MultiArrayLayout data
    global temp_task
    s_x = taskdata.sx
    s_y = taskdata.sy
    g_x = taskdata.gx
    g_y = taskdata.gy    
    tsg = taskdata.tsg
    temp_task = (s_x, s_y, g_x, g_y, tsg)
    print 'robot received temporary task <>(%s && <> %s)' %(str((s_x,s_y)), str((g_x,g_y)))
    return temp_task


def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp.secs > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp.secs
        # TODO: maybe add covariance check here?
        # print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose

def NaviControlCallback(twistdata):
    global navi_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.angular.z
    navi_control = [linear_v, angular_v]

def TeleControlCallback(twistdata):
    global tele_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.angular.z
    tele_control = [linear_v, angular_v]    
    

def SendGoal(GoalPublisher, goal, time_stamp):
    # goal: [x, y, yaw]
    GoalMsg = PoseStamped()
    #GoalMsg.header.seq = 0
    GoalMsg.header.stamp = time_stamp
    GoalMsg.header.frame_id = 'map'
    GoalMsg.pose.position.x = goal[0]
    GoalMsg.pose.position.y = goal[1]
    #GoalMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.pose.orientation.x = quaternion[0]
    GoalMsg.pose.orientation.y = quaternion[1]
    GoalMsg.pose.orientation.z = quaternion[2]
    GoalMsg.pose.orientation.w = quaternion[3]
    GoalPublisher.publish(GoalMsg)

def SendMix(MixPublisher, mix_control):
    MixMsg = Twist()
    MixMsg.linear.x = mix_control[0]
    MixMsg.linear.y = 0
    MixMsg.linear.z = 0
    MixMsg.angular.x = 0
    MixMsg.angular.y = 0
    MixMsg.angular.z = mix_control[1]
    MixPublisher.publish(MixMsg)


def hil_planner(sys_model, robot_name='turtlebot'):
    global robot_pose, navi_control, tele_control, temp_task
    robot_full_model, hard_task, soft_task = sys_model
    robot_pose = [0, [0, 0, 0]]
    navi_control = [0, 0]
    tele_control = [0, 0]
    mix_control = [0, 0]
    temp_task = None
    temp_task_s = False
    temp_task_g = False
    flag_task_incop = False
    rospy.init_node('ltl_planner_%s' %robot_name)
    print 'Robot %s: ltl_planner started!' %(robot_name)
    ###### publish to
    #----------
    #publish to
    #----------
    GoalPublisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 100)
    MixPublisher = rospy.Publisher('cmd_vel_mux/input/mix', Twist, queue_size = 100)
    #----------
    #subscribe to
    #----------
    # position estimate from amcl
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    # control command from amcl navigation
    rospy.Subscriber('cmd_vel_mux/input/navi', Twist, NaviControlCallback)
    # control command from tele operation
    rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, TeleControlCallback)
    # temporary task
    rospy.Subscriber('temp_task', task, TaskCallback)
    ####### robot information
    initial_beta = 0
    planner = ltl_planner(robot_full_model, hard_task, soft_task, initial_beta)
    ####### initial plan synthesis
    planner.optimal()
    print 'Original beta:', initial_beta
    print 'Initial optimal plan', planner.run.suf_plan
    #######
    reach_bound = 0.5 # m
    hi_bound = 0.1
    hi_bool = False
    hi_done = False
    reach_new = False
    #######
    robot_path = []
    reachable_prod_states = set(planner.product.graph['initial'])
    pre_reach_ts = None
    A_robot_pose = []
    A_control = []
    A_beta = []
    #######
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now()-t0
            print '----------Time: %.2f----------' %t.to_sec()
            A_robot_pose.append(list(robot_pose))            
            A_control.append([tele_control, navi_control, mix_control])
            # robot past path update
            reach_ts = planner.reach_ts_node(robot_pose[1], reach_bound)
            if ((reach_ts) and (reach_ts != pre_reach_ts)):
                print 'new region reached', reach_ts
                robot_path.append(reach_ts)
                reachable_prod_states = planner.update_reachable(reachable_prod_states, reach_ts)
                pre_reach_ts = reach_ts
                reach_new = True
            else:
                reach_new = False
            #------------------------------
            # mix control inputs
            if norm2(tele_control, [0,0]) >= hi_bound:
                print '--- Human inputs detected ---'
                hi_bool = True
                dist_to_trap = planner.prod_dist_to_trap(robot_pose[1], reachable_prod_states)
                if dist_to_trap >=0:
                    print 'Distance to trap states in product: %.2f' %dist_to_trap
                    mix_control, gain = smooth_mix(tele_control, navi_control, dist_to_trap)
                    SendMix(MixPublisher, mix_control)
                    print 'mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(mix_control, navi_control, tele_control, gain)
                else:
                    print 'No trap states are close'
                    dist_to_trap = 1000
                    mix_control, gain = smooth_mix(tele_control, navi_control, dist_to_trap)
                    SendMix(MixPublisher, mix_control)
                    print 'mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(mix_control, navi_control, tele_control, gain)
                rospy.sleep(0.2)
            else:
                print 'No Human inputs. Autonomous controller used.'
                mix_control = list(navi_control)
                SendMix(MixPublisher, mix_control)
            print 'robot_path:', robot_path
            # print 'reachable_prod_states', reachable_prod_states
            #------------------------------
            # estimate human preference, i.e. beta
            # and update discrete plan
            if ((reach_new) and (planner.start_suffix())):
                print 'robot_path:', robot_path
                print 'reachable_prod_states', reachable_prod_states
                if hi_bool:
                    print '------------------------------'
                    print '---------- In IRL mode now ----------'
                    est_beta_seq, match_score = planner.irl(robot_path, reachable_prod_states)
                    hi_bool = False
                    A_beta.append(est_beta_seq)
                    print '------------------------------'
                print '--- New suffix execution---'                
                robot_path = [reach_ts]
                reachable_prod_states = planner.intersect_accept(reachable_prod_states)
            #------------------------------
            # satisfy temporary task
            if temp_task:
                if not flag_task_incop:
                    planner.add_temp_task(temp_task)
                    flag_task_incop = True
                reg_s = (temp_task[0], temp_task[1])
                reg_g = (temp_task[2], temp_task[3])
                if ((reach_ts) and (reach_ts[0] == reg_s)):
                    temp_task_s = True
                    print 'robot reaches pi_s in the temp task:%s' %str(reg_s)
                if (temp_task_s) and ((reach_ts) and (reach_ts[0] == reg_g)):
                    temp_task_g = True
                    print 'robot reaches pi_g in the temp task:%s' %str(reg_g)
                if (temp_task_s) and (temp_task_g):
                    print 'robot accomplished temporary task <>(%s && <> %s)' %(str(reg_s), str(reg_g))
                    temp_task = None
                    temp_task_s = False
                    temp_task_g = False
                    flag_task_incop = False
            #------------------------------
            # plan execution
            current_goal = planner.next_move
            # next move is action
            if isinstance(current_goal, str):
                print 'the robot next_move is an action, currently not implemented for %s' %robot_name
                break
            # next move is motion
            if ((reach_ts) and (reach_ts[0] == current_goal)):    
                print('Goal %s reached by %s.' %(str(current_goal),str(robot_name)))
                planner.find_next_move()
            else:
                SendGoal(GoalPublisher, current_goal, t)
                print('Goal %s sent to %s.' %(str(current_goal),str(robot_name)))
                rospy.sleep(0.5)
        except rospy.ROSInterruptException:
            pickle.dump([A_robot_pose, A_control, A_beta], open('data/exp_long_cord.p', 'wb'))
            print 'exp_long_cord.p saved'
            print A_robot_pose
            pass
        pickle.dump([A_robot_pose, A_control, A_beta], open('data/exp_long_cord.p', 'wb'))
        print 'exp_long_cord.p saved'
        print A_robot_pose



if __name__ == '__main__':
    hil_planner(sys_model)
