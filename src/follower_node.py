#! /usr/bin/env python
'''
Adapted from Relaxed_IK work
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 7/1/18

Designed to subscribe to a topic containing end-effector pose of one hand, then calculating the pose of the other hand to achieve camera following.
'''
######################################################################################################
import os
import sys
import rospy
import relaxed_ik
from RelaxedIK.relaxedIK import RelaxedIK
from sensor_msgs.msg import JointState

# Add publisher to send solution to Gamepad_Action_Sequence 

#eepg = None
#def eePoseGoals_cb(data):
#    global eepg
#    eepg = data
left_joint_values = JointState()
left_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
right_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
pos_goals = []
quat_goals = []

def callback_pose(data):
     global left_joint_values
     left_joint_values.position = 7*[0.0]
     left_joint_values.name = left_joints
     left_joint_values.header = data.header
     for i,name in enumerate(left_joints):
        index = data.name.index(name)
        left_joint_values.position[i] = data.position[index]
        


if __name__ == '__main__':
    
    rospy.init_node('follower_node')
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointState ,queue_size=3)
    rospy.Subscriber('/robot/joint_states', JointState, callback_pose)

    config_file_name = 'second_victory.config' #rospy.get_param('config_file_name', default='second_victory.config')
    my_relaxedIK = RelaxedIK.init_from_config(config_file_name)
    num_chains = my_relaxedIK.vars.robot.numChains

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        header = left_joint_values.header
        xopt = my_relaxedIK.solve(pos_goals, quat_goals, left_joint_values.name, left_joint_values.position)
     
        right_arm_solution = JointState()
        right_arm_solution.header = header
        right_arm_solution.name = right_joints
        right_arm_solution.position = xopt[0:7]

        angles_pub.publish(right_arm_solution)

        rate.sleep()

