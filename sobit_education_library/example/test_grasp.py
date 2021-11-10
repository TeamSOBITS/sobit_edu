#!/usr/bin/env python3
#coding: utf-8

import rospy
from sobit_education_module import SobitEducationController
from sobit_education_module import SobitTurtlebotController
from sobit_education_module import Joint
import sys

def test(target_name):
    rospy.init_node('test')
    args = sys.argv
    edu_arm_pantilt_ctr = SobitEducationController(args[0]) # args[0] : C++上でros::init()を行うための引数
    edu_wheel_ctr = SobitTurtlebotController(args[0])       # args[0] : C++上でros::init()を行うための引数

    grasp_flag = False

    # 決められたポーズをする
    print("starting")
    edu_arm_pantilt_ctr.movePose( "detecting_pose" )
    rospy.sleep(5.0)

    # ハンドを動かす
    edu_arm_pantilt_ctr.moveJoint( Joint.HAND_MOTOR_JOINT, 1.57, 2.0, True )
    rospy.sleep(2.0)

    # 把持する対象の物体があった場合、
    # そこの位置までアームを移動させる
    # grasp_flag = edu_arm_pantilt_ctr.moveGripperToTarget(target_name, 0.0, 0.0, 0.0)
    grasp_flag = edu_arm_pantilt_ctr.moveGripperToTargetXYZ(0.5, 0.5, 0.75, 0.0, 0.0, 0.0)
    print("result : ", grasp_flag)
    rospy.sleep(2.0)
    
    # ハンドを動かす
    edu_arm_pantilt_ctr.moveJoint( Joint.HAND_MOTOR_JOINT, 0.0, 2.0, True )
    rospy.sleep(2.0)

    # 決められたポーズをする
    edu_arm_pantilt_ctr.movePose( "initial_pose" )

if __name__ == '__main__':
    try:
        test("pringles")
    except rospy.ROSInterruptException: pass
