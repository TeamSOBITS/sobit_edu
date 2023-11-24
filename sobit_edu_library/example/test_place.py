#!/usr/bin/env python3
#coding: utf-8

import sys
import rospy
from sobit_edu_module import SobitEduJointController
from sobit_edu_module import SobitEduWheelController
from sobit_edu_module import Joint


def test_place_on_table():
    rospy.init_node('sobit_edu_test_place_on_table')

    args = sys.argv
    edu_joint_ctrl = SobitEduJointController(args[0])
    edu_wheel_ctrl = SobitEduWheelController(args[0]) # args[0] : C++上でros::init()を行うための引数

    # Set the detecting_pose
    edu_joint_ctrl.moveToPose( "initial_pose" )

    """
    # Lauch the placeable_position_estimator node
    package_name = "sobit_pro_bringup"
    node_name    = "placeable_position_estimator"
    launch_name  = node_name + ".launch"
    comand       = "roslaunch " + package_name + " " + launch_name
    p = Popen(comand.split(), shell=True)
    p.wait()

    if( not p.returncode ){
        rospy.logerr("Failed to launch the placeable_position_estimator node");
    }

    # Turn on detection service
    rospy.wait_for_service('placeable_position_estimator/execute_ctrl')
    try:
        client = rospy.ServiceProxy('placeable_position_estimator/execute_ctrl', SetBool)
        client(True)
    except rospy.ServiceException as e:
        rospy.logerr( "Failed to call service placeable_position_estimator : %s" % e )

    rospy.sleep(10.0)
    """

    # Option 1: Place object on the given TF position
    # Arm will move down until it touches the placeable_point
    # is_done = edu_joint_ctrl.moveGripperToPlaceTF( "placeable_point", -0.15, 0.0, 0.2 )

    # Option 2: Place object on the given coordinates (x,y,z) position
    # Arm will move down until it touches the placeable_point
    is_done = edu_joint_ctrl.moveHandToPlaceCoord( 0.0, 0.0, 0.3, -0.15, 0.0, 0.2 )
    rospy.sleep(3.0)

    if( is_done ):
        # Open the gripper
        edu_joint_ctrl.moveJoint( Joint.HAND_JOINT, -1.57, 2.0, True )

        # Set the put_high_pose pose to avoid collision
        edu_joint_ctrl.moveToPose( "detecting_pose")
    else:
        rospy.logerr( "Failed to place the object" )

    # Set the initial pose
    edu_joint_ctrl.moveToPose( "initial_pose")

    del edu_joint_ctrl
    del edu_wheel_ctrl
    del args

    """
    # Kill the placeable_position_estimator node
    command = "rosnode kill /" + node_name + "/" + node_name + "_node"
    p = Popen(command.split(), shell=True)
    p.wait()

    if( not p.returncode ){
        rospy.logerr("Failed to kill the placeable_position_estimator node");
    }

    """

if __name__ == '__main__':
    try:
        test_place_on_table()
    except rospy.ROSInterruptException:
        pass
