#!/usr/bin/env python3
import rospy
from sobit_education_module import SobitEducationController
from sobit_education_module import Joint
from geometry_msgs.msg import Point
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    args = sys.argv
    edu_ctr = SobitEducationController(args[0]) # args[0] : C++上でros::init()を行うための引数

    ###    arm controll     ###
    ### arm_roll   =   0.00 ###
    ### arm_flex   =  -1.57 ###
    ### elbow_flex =   1.57 ###
    ### wrist_flex =   0.00 ###
    ### hand_motor =   -1.0 ###
    edu_ctr.moveArm( 0.0, -1.57, 1.57, 0.0, -1.0 )

    rospy.sleep(2.0)

    # 決められたポーズをする
    edu_ctr.movePose( "initial_pose" )

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
