#!/usr/bin/env python3
#coding: utf-8

import rospy
from sobit_edu_module import SobitEducationController
from sobit_edu_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    edu_ctr = SobitEducationController(args[0]) # args[0] : C++上でros::init()を行うための引数

    ###         arm controll       ###
    ### arm_shoulder_pan  =   0.00 ###
    ### arm_shoulder_tilt =  -1.5708 ###
    ### arm_elbow_tilt    =   1.5708 ###
    ### arm_wrist_tilt    =   0.00 ###
    ### hand              =   -1.0 ###
    # edu_ctr.moveJoint( Joint.HAND_JOINT, 1.0, 2.0, True )
    edu_ctr.moveArm( 0.0, -1.5708, 1.5708, 0.0, 1.0 )

    rospy.sleep(2.0)
    # edu_ctr.moveJoint( Joint.HAND_JOINT, 0, 2.0, True )


    # 決められたポーズをする
    edu_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
