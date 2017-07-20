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
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler


from init_sys import sys_model
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


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
    rospy.Subscriber('cmd_vel_mux/input/mix', Twist, TeleControlCallback)
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
    #######
    reach_bound = 0.5 # m
    hi_v_bound = 0.1 # m/s
    hi_w_bound = 0.1 # rad/s
    #######
    robot_path = []
    reachable_prod_states = set(planner.product['initial'])
    pre_reach_reg = None
    #######
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            t = rospy.Time.now()-t0
            print '----------Time: %.2f----------' %t.to_sec()
            # robot past path update
            reach_reg = planner.reach_ts_node(robot_pose, reach_bound):
            if ((reach_reg) and (reach_reg != pre_reach_reg)):
                robot_path.append(reach_reg)
                reachable_prod_states = planner.update_reachable(reachable_prod_states, reach_reg)
                pre_reach_reg = list(reach_reg)
            # plan execution
            current_goal = planner.next_move
            # next move is action
            if isinstance(current_goal, str):
                print 'the robot next_move is an action, currently not implemented for %s' %robot_name
                break
            # next move is motion
                
            if ((norm2(robot_pose[1][0:2], current_goal[0:2]) > reach_xy_bound) or (abs(robot_pose[1][2])-current_goal[2]) > reach_yaw_bound):
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
