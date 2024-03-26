#!/usr/bin/env python3
#coding: utf-8

import sys
import rospy
from sobit_edu_module import SobitEduJointController
from sobit_edu_module import SobitEduWheelController
from sobit_edu_module import Joint


def test_grasp_on_floor():
    rospy.init_node('sobit_edu_test_grasp_on_floor')

    args = sys.argv
    edu_joint_ctrl = SobitEduJointController(args[0])
    edu_wheel_ctrl = SobitEduWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # Set the detecting pose
    edu_joint_ctrl.moveToPose( "detecting_pose" )

    # Open the gripper
    edu_joint_ctrl.moveJoint( Joint.HAND_JOINT, 1.5708, 2.0, True )

    # Option 1: Grasp the target on the given TF position
    # Move the arm (inverse kinematics)
    # is_done = edu_joint_ctrl.moveGripperToTargetTF(target_name, -0.3, 0.0, 0.0)

    # """
    # Option 2: Grasp the target on the given coordinates (x,y,z) position
    # Move the arm (forward kinematics)
    is_done = edu_joint_ctrl.moveHandToTargetCoord(0.2, 0.2, 0.75, -0.3, 0.0, 0.0)
    # """

    # Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(0.3)
    
    # Close the gripper
    edu_joint_ctrl.moveJoint( Joint.HAND_JOINT, 0.0, 2.0, True )

    # Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(-0.3)

    # Check if grasped based on the force sensor
    is_done *= edu_joint_ctrl.graspDecision( 300, 1000 )

    if( is_done ):
        # Set the put_high_pose pose to avoid collision
        edu_joint_ctrl.moveToPose( "detecting_pose" )
    else:
        rospy.logerr("Failed to grasp the object")

    # Set the initial pose
    edu_joint_ctrl.moveToPose( "initial_pose" )

    del edu_joint_ctrl
    del edu_wheel_ctrl
    del args

if __name__ == '__main__':
    try:
        test_grasp_on_floor()
    except rospy.ROSInterruptException:
        pass