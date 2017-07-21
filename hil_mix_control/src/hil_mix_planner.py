#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('hil_mix_control')
import rospy
import sys

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist

from math import pi as PI
from math import atan2, sin, cos, sqrt, exp

from tf.transformations import euler_from_quaternion, quaternion_from_euler


from init_sys import sys_model
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


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
    mix_control[0] = navi_control[0] + gain*tele_control[0]
    mix_control[1] = navi_control[1] + gain*tele_control[1]
    return mix_control


def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp
        # TODO: maybe add covariance check here?
        # print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose

def NaviControlCallback(twistdata):
    global navi_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.linear.z
    navi_control = [linear_v, angular_v]

def TeleControlCallback(twistdata):
    global tele_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.linear.z
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
    global robot_pose
    robot_full_model, hard_task, soft_task = sys_model
    robot_pose = [None, init_pose]
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
    ####### robot information
    full_model = MotActModel(ts, act)
    initial_beta = 1
    planner = ltl_planner(robot_full_model, hard_task, soft_task, initial_beta)
    ####### initial plan synthesis
    planner.optimal()
    print 'Original beta:', initial_beta
    print 'Initial optimal plan', planner.run.suf_plan
    #######
    reach_bound = 0.5 # m
    hi_bound = 0.1
    hi_bool = False
    #######
    robot_path = []
    reachable_prod_states = set(planner.product['initial'])
    pre_reach_ts = None
    #######
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now()-t0
            print '----------Time: %.2f----------' %t.to_sec()
            # robot past path update
            reach_ts = planner.reach_ts_node(robot_pose, reach_bound):
            if ((reach_ts) and (reach_ts != pre_reach_ts)):
                robot_path.append(reach_ts)
                reachable_prod_states = planner.update_reachable(reachable_prod_states, reach_ts)
                pre_reach_ts = list(reach_ts)
            #------------------------------
            # mix control inputs
            if norm2(tele_control) >= hi_bound:
                print '--- Human inputs detected ---'
                hi_bool = True
                dist_to_trap = planner.prod_dist_to_trap(robot_pose, reachable_prod_states)
                if dist_to_trap >=0:
                    print 'Distance to trap states in product: %.2f' %dist_to_trap
                    mix_control = smooth_mix(tele_control, navi_control, dist_to_trap)
                    SendMix(MixPublisher, mix_control)
                else:
                    print 'No trap states are close'
                rospy.sleep(10)
            #------------------------------
            # estimate human preference, i.e. beta
            # and update discrete plan            
            if ((hi_bool) and (reach_ts)
                and (planner.check_accept(reachable_prod_states))):
                print '--- Accepting state reached ---'
                est_beta_seq, match_score = planner.irl(robot_path, reachable_prod_states)
                hi_bool = False
                planner.set_to_suffix()
                robot_path = []
            #------------------------------
            # plan execution
            current_goal = planner.next_move
            # next move is action
            if isinstance(current_goal, str):
                print 'the robot next_move is an action, currently not implemented for %s' %robot_name
                break
            # next move is motion
            if ((reach_reg) and (reach_reg[0] != current_goal)):    
                SendGoal(GoalPublisher, current_goal, t)
                print('Goal %s sent to %s.' %(str(current_goal),str(robot_name)))
                rospy.sleep(10)
            else:
                print('Goal %s reached by %s.' %(str(current_goal),str(robot_name)))
                planner.find_next_move()
        except rospy.ROSInterruptException:
            pass



if __name__ == '__main__':
    try:
        hil_planner(sys_model)
    except rospy.ROSInterruptException:
        pass
