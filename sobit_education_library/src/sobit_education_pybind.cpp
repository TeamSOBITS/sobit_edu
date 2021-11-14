#include <sobit_education_library/sobit_education_controller.hpp>

using namespace sobit_education;
namespace py = pybind11;

PYBIND11_MODULE(sobit_education_module, m) {
    py::enum_<Joint>( m, "Joint" )
        .value( "ARM_ROLL_JOINT", Joint::ARM_ROLL_JOINT )
        .value( "ARM_FLEX_JOINT_RGT", Joint::ARM_FLEX_JOINT_RGT )
        .value( "ARM_FLEX_JOINT_LFT", Joint::ARM_FLEX_JOINT_LFT )
        .value( "ELBOW_FLEX_JOINT", Joint::ELBOW_FLEX_JOINT )
        .value( "WRIST_FLEX_JOINT", Joint::WRIST_FLEX_JOINT )
        .value( "HAND_MOTOR_JOINT", Joint::HAND_MOTOR_JOINT )
        .value( "XTION_PAN_JOINT", Joint:: XTION_PAN_JOINT )
        .value( "XTION_TILT_JOINT", Joint::XTION_TILT_JOINT )
        .value( "JOINT_NUM", Joint::JOINT_NUM )
        .export_values();
    
    py::class_<SobitTurtlebotController>(m, "SobitTurtlebotController")
        .def( py::init< const std::string& >() )
        .def( "controlWheelLinear", &SobitTurtlebotController::controlWheelLinear, "control Wheel Linear" )
        .def( "controlWheelRotateRad", &SobitTurtlebotController::controlWheelRotateRad, "control Wheel Rotate Rad" )
        .def( "controlWheelRotateDeg", &SobitTurtlebotController::controlWheelRotateDeg, "control Wheel Rotate Deg" );
    
    py::class_<SobitEducationController, SobitTurtlebotController>(m, "SobitEducationController")
        .def( py::init< const std::string& >() )
        .def( "moveJoint", &SobitEducationController::moveJoint, "moveJoint", 
            py::arg("joint_num"), py::arg("rad"), py::arg("sec"), py::arg("is_sleep") = true )
        .def( "moveXtionPanTilt", &SobitEducationController::moveXtionPanTilt, "move Xtion PanTilt", 
            py::arg("pan_rad"), py::arg("tilt_rad"), py::arg("sec"), py::arg("is_sleep") = true )
        .def( "moveArm", &SobitEducationController::moveArm, "move Arm" )
        .def( "movePose", &SobitEducationController::movePose, "move Pose" )
        .def( "moveGripperToTarget", &SobitEducationController::moveGripperToTarget, "moveGripperToTarget" )
        .def( "moveGripperToTargetXYZ", &SobitEducationController::moveGripperToTargetXYZ, "moveGripperToTargetXYZ" );     
}