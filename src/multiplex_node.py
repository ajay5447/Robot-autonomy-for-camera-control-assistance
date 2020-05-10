#! /usr/bin/env python
'''

'''
######################################################################################################

from start_here import urdf_file_name, joint_names, joint_ordering, ee_fixed_joints, starting_config, \
    joint_state_define, collision_file_name, fixed_frame, config_file_name
from RelaxedIK.relaxedIK import RelaxedIK
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import rospy
import roslaunch
import os
import tf
import math



class MultiplexerInterface(object):
    def __init__(self):
        rospy.init_node('multiplex_node')
        self.control_state = None
        self.weight_array = []

        self.relaxedIK = RelaxedIK.init_from_config(config_file_name)

        self.left_js_pub = rospy.Publisher('/left_arm_controller/command', JointTrajectory, queue_size=1)
        self.right_js_pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=1)

        rospy.Subscriber('/controlState', String, self.cb_control_state,queue_size=1)
        # rospy.Subscriber('/uiJointpose', JointState, self.cb_joint_control,queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.cb_joint_control,queue_size=1)
        rospy.Subscriber('/uiCartesianPose', Pose, self.cb_cartesion_control,queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.cb_objective_control,queue_size=1)

    def cb_joint_control(self,data):
        if not self.control_state == 'joint':
            return

        left_arm_names = data.name[:7]
        left_arm_positions = data.position[:7]

        right_arm_names = data.name[7:]
        right_arm_positions = data.position[:7]


        left_msg = self.make_JointTrajectory_from_joints(vals=left_arm_positions,names=left_arm_names)
        right_msg = self.make_JointTrajectory_from_joints(vals=right_arm_positions,names=right_arm_names)

        # self.left_js_pub.publish(left_msg)
        self.right_js_pub.publish(right_msg)

        return


    def cb_cartesion_control(self,data):
        if not self.control_state == 'cartesian':
            return

    def cb_objective_control(self,data):
        if not self.control_state == 'objective':
            return



    def cb_control_state(self, data):

        if data.data == self.control_state:
            return
        else:
            self.control_state = data.data

        if data.data == 'joint':
            self.weight_array = [0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0,
                                 0, 0]
        elif data.data == 'cartesian':
            self.weight_array = [0, 0, 0, 0, 0,
                                 1, 1, 1, 1, 1,
                                 1, 1]
        elif data.data == 'objective':
            self.weight_array = [1, 1, 1, 1, 1,
                                 1, 1, 1, 1, 1,
                                 0, 0]

        for ind, wi in enumerate(self.weight_array):
            self.relaxedIK.vars.weight_funcs[ind].set_value(wi)

        return

    def make_JointTrajectory_from_joints(self, vals=[],names=[]):
        if not len(vals) == len(names):
            print("ERROR: length miss match")
            return None

        JT_msg = JointTrajectory()
        JT_msg.joint_names = names



        JTP_msg = JointTrajectoryPoint()
        JT_msg.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0)
        JTP_msg.time_from_start = rospy.Duration.from_sec(5.0)

        JTP_msg.positions = vals
        JTP_msg.velocities = [0.0]*len(names)
        JTP_msg.accelerations = [0.0]*len(names)
        JTP_msg.effort = [0.0]*len(names)

        JT_msg.points.append(JTP_msg)

        return JT_msg


def main():
    try:

        multiplexer = MultiplexerInterface()

        rospy.spin()


        print "============ End"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
