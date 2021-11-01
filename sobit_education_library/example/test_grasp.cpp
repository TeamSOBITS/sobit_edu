#include "ros/ros.h"
#include <sobit_education_library/sobit_education_controller.hpp>
#include <sobit_education_library/sobit_turtlebot_controller.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_edu_grasp");

    sobit_education::SobitEducationController edu_arm_ctr;
    sobit_education::SobitEducationController edu_wheel_ctr;

    double grasp_flag = false;

    // Pose: detecting_pose
    std::cout << "starting" << std:: endl;
    edu_arm_ctr.movePose("detecting_pose");
    ros::Duration(5.0).sleep();

    // Open the hand
    edu_arm_ctr.moveJoint( sobit_education::Joint::HAND_MOTOR_JOINT, 1.57, 2.0, true );
    ros::Duration(2.0).sleep();

    // Move the hand towards the target "pringles"
    // grasp_flag = edu_arm_ctr.moveGripperToTarget("pringles", 0., 0., 0.);
    grasp_flag = edu_arm_ctr.moveGripperToTargetXYZ(0.5, 0.5, 0.5, 0., 0., 0.);
    std::cout << "Is grasped? " << (grasp_flag? "True":"False") << std::endl;
    ros::Duration(2.0).sleep();

    // Close hand
    edu_arm_ctr.moveJoint( sobit_education::Joint::HAND_MOTOR_JOINT, 0.0, 2.0, true );
    ros::Duration(2.0).sleep();

    // Move the arm to a high position and then to the initial pose
    edu_arm_ctr.movePose( "initial_pose" );

    return 0;
}
