#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
# from tf.transformations import euler_from_quaternion
from pid import PID
import math
import numpy as np
from param import *
from planer import TrapezoidalPlanner

linear_controller_x = PID(1.5, 0, 9)
linear_controller_y = PID(1.5, 0, 9)

last_robot_state = np.array([0.0,0.0,0.0,0.0], dtype=float) # x. y ,x_dot, y_dot
robot_state = np.array([0.0,0.0,0.0,0.0], dtype=float) # x. y ,x_dot, y_dot
target_state = np.array([0.0,0.0,0.0,0.0], dtype=float) # x, y ,x_dot, y_dot

vel = 1.0
acc = 1.0
trac_traj = np.array([
    [1.0,0.0,0.0,0.0],
    [1.0,1.0,0.0,0.0],
    [-1.0,1.0,0.0,0.0],
    [-1.0,0.0,0.0,0.0],
    [1.0,0.0,0.0,0.0],
    [1.0,-1.0,0.0,0.0],
    [-1.0,-1.0,0.0,0.0],
    [-1.0,0.0,0.0,0.0],
], dtype=float)

def callback_robot_state(data):
    global robot_state
    global last_robot_state
    last_robot_state = robot_state.copy()
    robot_state[0] = data.pose.position.x
    robot_state[1] = data.pose.position.y
    robot_state[2] = (robot_state[0] - last_robot_state[0]) / dt
    robot_state[3] = (robot_state[1] - last_robot_state[1]) / dt
    
    # rospy.loginfo("robot_state: %f, %f, last_robot_state: %f, %f\r\n", robot_state[0], robot_state[1], last_robot_state[0], last_robot_state[1])
    # rospy.loginfo("robot_state: %f, %f, %f, %f\r\n", robot_state[0], robot_state[1], robot_state[2], robot_state[3])

def controller():
    global robot_state
    global target_state
    rospy.init_node('talker',anonymous=True)
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
    rospy.Subscriber('/robot/esti_model_state', ModelState, callback_robot_state)
    twist = Twist()
    F = np.array([0.0,0.0])
    planner = TrapezoidalPlanner(max_speed=vel, accel=acc)
    planner.update_target(robot_state[0],0.4)
    
    #============design your trace below=============
    rate = rospy.Rate(10)
    state = 0
    target_state = trac_traj[state].copy()
    deadband = 0.005
    slowband = 0.04
    while not rospy.is_shutdown():
        if math.sqrt((robot_state[0] - target_state[0])**2 + (robot_state[1] - target_state[1])**2) < 0.02:
            state += 1
            state = state % len(trac_traj)
            target_state = trac_traj[state].copy()
        
        target_state = trac_traj[state].copy()

        # linear_controller_x.update(robot_state[0], target_state[0])
        # F[0] = linear_controller_x.get_output()
        # linear_controller_y.update(robot_state[1], target_state[1])
        # F[1] = linear_controller_y.get_output()
        target_state_temp = target_state.copy()
        # target_state_temp[2] = (target_state_temp[0] - robot_state[0]) / (10) + target_state_temp[2]
        # target_state_temp[3] = (target_state_temp[1] - robot_state[1]) / (10) + target_state_temp[3]
        
        # target_pos, target_vel = planner.get_reference()
        # target_state_temp = np.array([target_pos, 0.0, target_vel, 0.0], dtype=float)
        
        x_diff = target_state_temp[0] - robot_state[0]
        y_diff = target_state_temp[1] - robot_state[1]
        
        # rospy.loginfo("x_diff: %.3f, y_diff: %.3f\r\n", x_diff, y_diff)
        
        if math.fabs(x_diff) < deadband :
            target_state_temp[2] = 0
        elif math.fabs(x_diff) < slowband:
            target_state_temp[2] = x_diff * vel
        else:
            target_state_temp[2] = sign(x_diff) * vel
        if math.fabs(y_diff) < deadband :
            target_state_temp[3] = 0
        elif math.fabs(y_diff) < slowband:
            target_state_temp[3] = y_diff * vel
        else:
            target_state_temp[3] = sign(y_diff) * vel
        
        F = np.dot(-K_lqr, robot_state - target_state_temp)
        
        twist.linear.x = F[0]
        twist.angular.z = F[1]
        # rospy.loginfo("robot_state: %.3f, %.3f, %.3f, %.3f\r\n", robot_state[0], robot_state[1], robot_state[2], robot_state[3])
        rospy.loginfo("target_state: %.3f, %.3f, %.3f, %.3f\r\n", target_state_temp[0], target_state_temp[1], target_state_temp[2], target_state_temp[3])
        rospy.loginfo("state: %d, robot_state: %.3f, %.3f, output: %.3f, %.3f\r\n",state, robot_state[0], robot_state[1], F[0], F[1])
        pub.publish(twist)
        rate.sleep()
    sys.exit(0)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


























