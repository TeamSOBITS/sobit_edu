#include "ros/ros.h"
#include <sobit_education_library/sobit_education_controller.hpp>
#include <sobit_education_library/sobit_turtlebot_controller.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_edu_grasp");

    sobit_education::SobitEducationController edu_arm_ctr;
    sobit_education::SobitEducationController edu_wheel_ctr;

    double grasp_flag = false;
    std::string target_name = "pringle";

    // Pose: detecting_pose
    std::cout << "starting" << std:: endl;
    edu_arm_ctr.moveToPose("detecting_pose");
    ros::Duration(3.0).sleep();

    // Open the hand
    edu_arm_ctr.moveJoint( sobit_education::Joint::HAND_MOTOR_JOINT, 1.57, 2.0, true );
    ros::Duration(2.0).sleep();

    // Move the hand towards the target "target_name"
    // grasp_flag = edu_arm_ctr.moveGripperToTarget(target_name, 0.0, 0.0, 0.0)
    // Caution: We shift the x axis target posiition -0.3 to avoid collision
    grasp_flag = edu_arm_ctr.moveGripperToTargetTF(target_name, -0.3, 0.0, 0.0);
    grasp_flag = edu_arm_ctr.moveGripperToTargetCoord(0.5, 0.5, 0.5, -0.3, 0.0, 0.0);
    std::cout << "Is grasped? " << (grasp_flag? "True":"False") << std::endl;
    ros::Duration(2.0).sleep();

    // Move forwards the robot (0.3m)
    edu_wheel_ctr.controlWheelLinear(0.3);
    ros::Duration(2.0).sleep();

    // Close hand
    edu_arm_ctr.moveJoint( sobit_education::Joint::HAND_MOTOR_JOINT, 0.0, 2.0, true );
    ros::Duration(2.0).sleep();

    // Move back the robot (-0.3m)
    edu_wheel_ctr.controlWheelLinear(-0.3);
    ros::Duration(2.0).sleep();

    // Pose: "initial pose"
    edu_arm_ctr.moveToPose( "initial_pose" );

    return 0;
}
