#!/usr/bin/env python3
import rospy
from sobit_edu_module import SobitTurtlebotController
import sys

def test():
    rospy.init_node('test')
    args = sys.argv
    edu_wheel_ctr = SobitTurtlebotController(args[0]) # args[0] : C++上でros::init()を行うための引数
    
    # タイヤ車輪をを動かす
    edu_wheel_ctr.controlWheelLinearFixed(1.5)
    rospy.sleep(5)
    edu_wheel_ctr.controlWheelRotateRad(1.5708)
    edu_wheel_ctr.controlWheelRotateDeg(-90)

    edu_wheel_ctr.controlWheelLinearFixed(-1.5)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
