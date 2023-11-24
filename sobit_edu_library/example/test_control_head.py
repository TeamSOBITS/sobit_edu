#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from sobit_edu_module import SobitEduJointController
from sobit_edu_module import Joint


def test_control_head():
    rospy.init_node('sobit_edu_test_control_head')

    args = sys.argv
    edu_joint_ctrl = SobitEduJointController(args[0])

    # r = rospy.Rate(1) # 10hz
    # ang = 0.8
    # args = sys.argv
    # edu_pantilt_ctr = SobitEduController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # while not rospy.is_shutdown():
    #     ang = -1.0 * ang

    #     # カメラパンチルトを動かす
    #     edu_pantilt_ctr.moveJoint( Joint.HEAD_CAMERA_PAN_JOINT, ang, 2.0, False )
    #     r.sleep()

    MAX_ANGLE    =  1.57
    MIN_ANGLE    = -1.57
    target_angle =  0.0
    increment    =  0.05

    while not rospy.is_shutdown():
        print(target_angle)
        target_angle += increment
        if target_angle > MAX_ANGLE or target_angle < MIN_ANGLE:
            increment *= -1.0

        # Option 1: Move the head joints simultaneously
        edu_joint_ctrl.moveHeadPanTilt( target_angle, target_angle, 0.5, False )
        rospy.sleep(0.5)

        """
        # Option 2: Move the head joints individually
        edu_joint_ctrl.moveJoint( Joint.HEAD_CAMERA_PAN_JOINT , target_angle, 0.5, True )
        edu_joint_ctrl.moveJoint( Joint.HEAD_CAMERA_TILT_JOINT, target_angle, 0.5, True )
        """

    del pro_joint_ctrl
    del args

if __name__ == '__main__':
    try:
        test_control_head()
    except rospy.ROSInterruptException:
        pass