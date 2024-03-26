#include <ros/ros.h>
#include "sobit_edu_library/sobit_edu_joint_controller.hpp"


int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_edu_test_control_arm");
    
    sobit_edu::SobitEduJointController edu_joint_ctrl;

    // Move all the arm joints
    edu_joint_ctrl.moveArm( 0.0, -1.5708, 1.5708, 0.0, 1.0 );

    // Open the hand
    edu_joint_ctrl.moveJoint( sobit_edu::Joint::HAND_JOINT, -1.57, 5.0, true );

    // Set the initial pose
    edu_joint_ctrl.moveToPose( "initial_pose", 5.0);

    return 0;
}