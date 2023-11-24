#include "sobit_edu_library/sobit_edu_library.h"
#include "sobit_edu_library/sobit_edu_joint_controller.hpp"
#include "sobit_edu_library/sobit_edu_wheel_controller.hpp"

using namespace sobit_edu;
namespace py = pybind11;

PYBIND11_MODULE( sobit_edu_module, m ) {
    py::enum_<Joint>( m, "Joint" )
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
    
    py::class_<SobitEduWheelController>( m, "SobitEduWheelController" )
        .def( py::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitEduWheelController::controlWheelLinear, "control Wheel Linear", 
            py::arg( "distance" ) )
        .def( "controlWheelRotateRad", &SobitEduWheelController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            py::arg( "angle_rad" ) )
        .def( "controlWheelRotateDeg", &SobitEduWheelController::controlWheelRotateDeg, "control Wheel Rotate Deg",
            py::arg( "angle_deg" ) );

    py::class_<SobitEduJointController>( m, "SobitEduJointController" )
        .def( py::init< const std::string& >() )
        .def( "moveToPose", &SobitEduJointController::moveToPose, "move Pose",
            py::arg( "pose_name" ),
            py::arg( "sec" ) = 5.0 )
        .def( "moveJoint", &SobitEduJointController::moveJoint, "moveJoint", 
            py::arg( "joint_num" ),
            py::arg( "rad" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveHeadPanTilt", &SobitEduJointController::moveHeadPanTilt, "move Head PanTilt", 
            py::arg( "pan_rad" ),
            py::arg( "tilt_rad" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveArm", &SobitEduJointController::moveArm, "move Arm",
            py::arg( "arm_shoulder_pan" ),
            py::arg( "arm_shoulder_tilt" ),
            py::arg( "arm_elbow_tilt" ),
            py::arg( "arm_wrist_tilt" ),
            py::arg( "hand" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveHandToTargetCoord", &SobitEduJointController::moveHandToTargetCoord, "moveHandToTargetCoord",
            py::arg( "goal_position_x" ),
            py::arg( "goal_position_y" ),
            py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveHandToTargetTF", &SobitEduJointController::moveHandToTargetTF, "moveHandToTargetTF",
            py::arg( "target_name" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveHandToPlaceCoord", &SobitEduJointController::moveHandToPlaceCoord, "move Hand To Placeable Position Coordinate",
            py::arg( "goal_position_x" ),
            py::arg( "goal_position_y" ),
            py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveHandToPlaceTF", &SobitEduJointController::moveHandToPlaceTF, "move Hand To Placeable Position TF",
            py::arg( "target_name" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "graspDecision", &SobitEduJointController::graspDecision, "grasp Decision",
            pybind11::arg( "min_curr" ) = 300, 
            pybind11::arg( "max_curr" ) = 1000)
        .def( "placeDecision", &SobitEduJointController::placeDecision, "place Decision",
            pybind11::arg( "min_curr" ) = 500, 
            pybind11::arg( "max_curr" ) = 1000);
}