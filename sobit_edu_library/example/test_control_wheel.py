#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import math
import rospy
from sobit_edu_module import SobitEduWheelController
from geometry_msgs.msg import Twist

def test_control_wheel():
    rospy.init_node('sobit_edu_test_control_wheel')

    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size=1)
    vel = Twist()
    vel.linear.x = 0.1

    for angular in np.arange(-math.pi/2, math.pi/2, 0.01):
        vel.angular.z = angular
        print("linear = " + str(vel.linear.x) + "\tangular = " + str(vel.angular.z))
        pub.publish(vel)
        rospy.sleep(0.1)
    
    for angular in np.arange(math.pi/2, -math.pi/2, 0.01):
        vel.angular.z = angular
        print("linear = " + str(vel.linear.x) + "\tangular = " + str(vel.angular.z))
        pub.publish(vel)
        rospy.sleep(0.1)
    
    zero_vel = Twist()
    pub.publish(zero_vel)

    """
    args = sys.argv
    edu_wheel_ctrl = SobitEduWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(1.5)

    # Move the wheels (rotational motion: Radian)
    edu_wheel_ctrl.controlWheelRotateRad(1.5708)

    # Move the wheels (rotational motion: Degree)
    edu_wheel_ctrl.controlWheelRotateDeg(-90)

    # Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(-1.5)

    del edu_wheel_ctrl
    del args
    """
    

if __name__ == '__main__':
    try:
        test_control_wheel()
    except rospy.ROSInterruptException:
        pass
