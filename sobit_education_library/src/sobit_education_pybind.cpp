#include <sobit_education_library/sobit_education_controller.hpp>

using namespace sobit_education;
namespace py = pybind11;

PYBIND11_MODULE( sobit_education_module, m ) {
    py::enum_<Joint>( m, "Joint" )
        .value( "ARM_ROLL_JOINT", Joint::ARM_ROLL_JOINT )
        .value( "ARM_FLEX_JOINT_RGT", Joint::ARM_FLEX_JOINT_RGT )
        .value( "ARM_FLEX_JOINT_LFT", Joint::ARM_FLEX_JOINT_LFT )
        .value( "ELBOW_FLEX_JOINT_RGT", Joint::ELBOW_FLEX_JOINT_RGT )
        .value( "ELBOW_FLEX_JOINT_LFT", Joint::ELBOW_FLEX_JOINT_LFT )
        .value( "WRIST_FLEX_JOINT", Joint::WRIST_FLEX_JOINT )
        .value( "HAND_MOTOR_JOINT", Joint::HAND_MOTOR_JOINT )
        .value( "XTION_PAN_JOINT", Joint:: XTION_PAN_JOINT )
        .value( "XTION_TILT_JOINT", Joint::XTION_TILT_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values( );
    
    py::class_<SobitEducationController, SobitTurtlebotController>( m, "SobitEducationController" )
        .def( py::init< const std::string& >() )
        .def( "moveToPose", &SobitEducationController::moveToPose, "move Pose",
            py::arg( "pose_name" ),
            py::arg( "sec" ) = 5.0 )
        .def( "moveJoint", &SobitEducationController::moveJoint, "moveJoint", 
            py::arg( "joint_num" ),
            py::arg( "rad" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveHeadPanTilt", &SobitEducationController::moveHeadPanTilt, "move Head PanTilt", 
            py::arg( "pan_rad" ),
            py::arg( "tilt_rad" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveArm", &SobitEducationController::moveArm, "move Arm",
            py::arg( "arm_roll" ),
            py::arg( "arm_flex" ),
            py::arg( "elbow_flex" ),
            py::arg( "wrist_flex" ),
            py::arg( "hand_motor" ),
            py::arg( "sec" ) = 5.0,
            py::arg( "is_sleep" ) = true )
        .def( "moveGripperToTargetCoord", &SobitEducationController::moveGripperToTargetCoord, "moveGripperToTargetCoord",
            py::arg( "goal_position_x" ),
            py::arg( "goal_position_y" ),
            py::arg( "goal_position_z" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "moveGripperToTargetTF", &SobitEducationController::moveGripperToTargetTF, "moveGripperToTargetTF",
            py::arg( "target_name" ),
            py::arg( "diff_goal_position_x" ),
            py::arg( "diff_goal_position_y" ),
            py::arg( "diff_goal_position_z" ) )
        .def( "graspDecision", &SobitEducationController::graspDecision, "grasp Decision" );

    py::class_<SobitTurtlebotController>( m, "SobitTurtlebotController" )
        .def( py::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitTurtlebotController::controlWheelLinear, "control Wheel Linear", 
            py::arg( "distance" ) )
        .def( "controlWheelRotateRad", &SobitTurtlebotController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            py::arg( "angle_rad" ) )
        .def( "controlWheelRotateDeg", &SobitTurtlebotController::controlWheelRotateDeg, "control Wheel Rotate Deg",
            py::arg( "angle_deg" ) );
}