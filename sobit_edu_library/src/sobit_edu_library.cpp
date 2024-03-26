#include "sobit_edu_library/sobit_edu_library.h"
#include "sobit_edu_library/sobit_edu_joint_controller.hpp"
#include "sobit_edu_library/sobit_edu_wheel_controller.hpp"

using namespace sobit_edu;
// namespace pybind11 = pybind11bind11;

PYBIND11_MODULE( sobit_edu_module, m ) {
    pybind11::enum_<Joint>( m, "Joint" )
        .value( "ARM_SHOULDER_PAN_JOINT", Joint::ARM_SHOULDER_PAN_JOINT )
        .value( "ARM_SHOULDER_1_TILT_JOINT", Joint::ARM_SHOULDER_1_TILT_JOINT )
        .value( "ARM_SHOULDER_2_TILT_JOINT", Joint::ARM_SHOULDER_2_TILT_JOINT )
        .value( "ARM_ELBOW_1_TILT_JOINT", Joint::ARM_ELBOW_1_TILT_JOINT )
        .value( "ARM_ELBOW_2_TILT_JOINT", Joint::ARM_ELBOW_2_TILT_JOINT )
        .value( "ARM_WRIST_TILT_JOINT", Joint::ARM_WRIST_TILT_JOINT )
        .value( "HAND_JOINT", Joint::HAND_JOINT )
        .value( "HEAD_CAMERA_PAN_JOINT", Joint:: HEAD_CAMERA_PAN_JOINT )
        .value( "HEAD_CAMERA_TILT_JOINT", Joint::HEAD_CAMERA_TILT_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values( );
    
    pybind11::class_<SobitEduJointController>( m, "SobitEduJointController" )
        .def( pybind11::init< const std::string& >() )
        .def( "moveToPose", &SobitEduJointController::moveToPose, "move Pose",
            pybind11::arg( "pose_name" ),
            pybind11::arg( "sec" ) = 5.0 )
        .def( "moveAllJoint", &SobitEduJointController::moveAllJoint, "move all Joint",
            pybind11::arg( "arm_shoulder_pan" ),
            pybind11::arg( "arm_shoulder_tilt" ),
            pybind11::arg( "arm_elbow_tilt" ),
            pybind11::arg( "arm_wrist_tilt" ),
            pybind11::arg( "hand" ),
            pybind11::arg( "head_camera_pan" ),
            pybind11::arg( "head_camera_tilt" ),
            pybind11::arg( "sec" ) = 5.0, 
            pybind11::arg( "is_sleep" ) = true)
        .def( "moveJoint", &SobitEduJointController::moveJoint, "moveJoint", 
            pybind11::arg( "joint_num" ),
            pybind11::arg( "rad" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveHeadPanTilt", &SobitEduJointController::moveHeadPanTilt, "move Head PanTilt", 
            pybind11::arg( "pan_rad" ),
            pybind11::arg( "tilt_rad" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveArm", &SobitEduJointController::moveArm, "move Arm",
            pybind11::arg( "arm_shoulder_pan" ),
            pybind11::arg( "arm_shoulder_tilt" ),
            pybind11::arg( "arm_elbow_tilt" ),
            pybind11::arg( "arm_wrist_tilt" ),
            pybind11::arg( "hand" ),
            pybind11::arg( "sec" ) = 5.0,
            pybind11::arg( "is_sleep" ) = true )
        .def( "moveHandToTargetCoord", &SobitEduJointController::moveHandToTargetCoord, "moveHandToTargetCoord",
            pybind11::arg( "goal_position_x" ),
            pybind11::arg( "goal_position_y" ),
            pybind11::arg( "goal_position_z" ),
            pybind11::arg( "diff_goal_position_x" ),
            pybind11::arg( "diff_goal_position_y" ),
            pybind11::arg( "diff_goal_position_z" ) )
        .def( "moveHandToTargetTF", &SobitEduJointController::moveHandToTargetTF, "moveHandToTargetTF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "diff_goal_position_x" ),
            pybind11::arg( "diff_goal_position_y" ),
            pybind11::arg( "diff_goal_position_z" ) )
        .def( "moveHandToPlaceCoord", &SobitEduJointController::moveHandToPlaceCoord, "move Hand To Placeable Position Coordinate",
            pybind11::arg( "goal_position_x" ),
            pybind11::arg( "goal_position_y" ),
            pybind11::arg( "goal_position_z" ),
            pybind11::arg( "diff_goal_position_x" ),
            pybind11::arg( "diff_goal_position_y" ),
            pybind11::arg( "diff_goal_position_z" ) )
        .def( "moveHandToPlaceTF", &SobitEduJointController::moveHandToPlaceTF, "move Hand To Placeable Position TF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "diff_goal_position_x" ),
            pybind11::arg( "diff_goal_position_y" ),
            pybind11::arg( "diff_goal_position_z" ) )
        .def( "graspDecision", &SobitEduJointController::graspDecision, "grasp Decision",
            pybind11::arg( "min_curr" ) = 300, 
            pybind11::arg( "max_curr" ) = 1000)
        .def( "placeDecision", &SobitEduJointController::placeDecision, "place Decision",
            pybind11::arg( "min_curr" ) = 500, 
            pybind11::arg( "max_curr" ) = 1000);

    pybind11::class_<SobitEduWheelController>( m, "SobitEduWheelController" )
        .def( pybind11::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitEduWheelController::controlWheelLinear, "control Wheel Linear", 
            pybind11::arg( "distance" ) )
        .def( "controlWheelRotateRad", &SobitEduWheelController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            pybind11::arg( "angle_rad" ) )
        .def( "controlWheelRotateDeg", &SobitEduWheelController::controlWheelRotateDeg, "control Wheel Rotate Deg",
            pybind11::arg( "angle_deg" ) )
        .def( "rad2Deg", &SobitEduWheelController::rad2Deg, "rad 2 Deg", 
            pybind11::arg( "rad" ) )
        .def( "deg2Rad", &SobitEduWheelController::deg2Rad, "deg 2 Rad", 
            pybind11::arg( "deg" ) );

}