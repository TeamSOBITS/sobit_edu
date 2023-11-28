#include <ros/ros.h>
#include "sobit_edu_library/sobit_edu_wheel_controller.hpp"

int main( int argc, char *argv[] ){
    ros::init(argc, argv, "sobit_edu_test_control_wheel");

    sobit_edu::SobitEduWheelController edu_wheel_ctrl;

    // Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(1.5);
    ros::Duration(3.0).sleep();

    // Move the wheels (rotational motion: Radian)
    edu_wheel_ctrl.controlWheelRotateRad(1.5708);
    ros::Duration(3.0).sleep();

    // Move the wheels (rotational motion: Degree)
    edu_wheel_ctrl.controlWheelRotateDeg(-90);
    ros::Duration(3.0).sleep();

    // Move the wheels (linear motion)
    edu_wheel_ctrl.controlWheelLinear(-1.5);
    ros::Duration(3.0).sleep();
    
    return 0;
}
    
    
    
    
    
    
    
    

