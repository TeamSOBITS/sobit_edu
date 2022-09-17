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

    # 決められたポーズをする
    edu_ctr.moveToPose( "pan_pose" )
    rospy.sleep(5)

    # 決められたポーズをする
    edu_ctr.moveToPose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
