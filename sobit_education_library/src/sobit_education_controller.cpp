#include <sobit_education_library/sobit_education_controller.hpp>

using namespace sobit_education;

SobitEducationController::SobitEducationController( const std::string &name ) : SobitTurtlebotController( name ) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_xtion_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/xtion_trajectory_controller/command", 1);
    loadPose();
}

SobitEducationController::SobitEducationController( ) : SobitTurtlebotController( ) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_trajectory_controller/command", 1);
    pub_xtion_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/xtion_trajectory_controller/command", 1);    
    loadPose();
}

void SobitEducationController::loadPose() {
    XmlRpc::XmlRpcValue pose_val;
    if ( !nh_.hasParam("/education_pose") ) return; 
    nh_.getParam("/education_pose", pose_val);
    int pose_num = pose_val.size();
    pose_list_.clear();
    for ( int i = 0; i < pose_num; i++ ) {
        Pose pose;
        std::vector<double> joint_val(7, 0.0);
        pose.pose_name = static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::ARM_ROLL_JOINT] = static_cast<double>(pose_val[i]["arm_roll_joint"]); 
        joint_val[Joint::ARM_FLEX_JOINT] = static_cast<double>(pose_val[i]["arm_flex_joint"]); 
        joint_val[Joint::ELBOW_FLEX_JOINT] = static_cast<double>(pose_val[i]["elbow_flex_joint"]); 
        joint_val[Joint::WRIST_FLEX_JOINT] = static_cast<double>(pose_val[i]["wrist_flex_joint"]); 
        joint_val[Joint::HAND_MOTOR_JOINT] = static_cast<double>(pose_val[i]["hand_motor_joint"]); 
        joint_val[Joint::XTION_PAN_JOINT] = static_cast<double>(pose_val[i]["xtion_tilt_joint"]); 
        joint_val[Joint::XTION_TILT_JOINT] = static_cast<double>(pose_val[i]["xtion_pan_joint"]); 
        pose.joint_val = joint_val;
        pose_list_.push_back( pose );
    }
    return;
}

bool SobitEducationController::moveAllJoint( 
    const double arm_roll, 
    const double arm_flex, 
    const double elbow_flex, 
    const double wrist_flex, 
    const double hand_motor, 
    const double xtion_pan, 
    const double xtion_tilt, 
    const double sec, 
    bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        trajectory_msgs::JointTrajectory xtion_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::ARM_ROLL_JOINT], arm_roll, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT], arm_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT], elbow_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::WRIST_FLEX_JOINT], wrist_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_MOTOR_JOINT], hand_motor, sec, &arm_joint_trajectory );
        setJointTrajectory( joint_names_[Joint::XTION_PAN_JOINT], xtion_pan, sec, &xtion_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::XTION_TILT_JOINT], xtion_tilt, sec, &xtion_joint_trajectory );
        checkPublishersConnection ( pub_arm_control_ );
        checkPublishersConnection ( pub_xtion_control_ );
        pub_arm_control_.publish( arm_joint_trajectory );
        pub_xtion_control_.publish( xtion_joint_trajectory );
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitEducationController::moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        if( joint_num < Joint::XTION_PAN_JOINT ) {
            checkPublishersConnection ( pub_arm_control_ );
            pub_arm_control_.publish( joint_trajectory );
        } else {
            checkPublishersConnection ( pub_xtion_control_ );
            pub_xtion_control_.publish( joint_trajectory );
        }
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitEducationController::moveXtionPanTilt( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::XTION_PAN_JOINT], pan_rad, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::XTION_TILT_JOINT], tilt_rad, sec, &joint_trajectory );
        checkPublishersConnection ( pub_xtion_control_ );
        pub_xtion_control_.publish( joint_trajectory );
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitEducationController::movePose( const std::string &pose_name ) {
    bool is_find = false;
    std::vector<double> joint_val;
    for ( auto& pose : pose_list_ ) {
        if ( pose_name != pose.pose_name ) continue;
        is_find = true;
        joint_val = pose.joint_val;
        break;
    }
    if ( is_find ) {
        ROS_INFO("I found a '%s'", pose_name.c_str() );
        return moveAllJoint( 
                            joint_val[Joint::ARM_ROLL_JOINT], 
                            joint_val[Joint::ARM_FLEX_JOINT], 
                            joint_val[Joint::ELBOW_FLEX_JOINT], 
                            joint_val[Joint::WRIST_FLEX_JOINT], 
                            joint_val[Joint::HAND_MOTOR_JOINT], 
                            joint_val[Joint:: XTION_PAN_JOINT], 
                            joint_val[Joint::XTION_TILT_JOINT],
                            5.0 );
    } else {
        ROS_ERROR("'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitEducationController::moveArm ( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::ARM_ROLL_JOINT], arm_roll, 5.0, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT], arm_flex, 5.0, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT], elbow_flex, 5.0, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::WRIST_FLEX_JOINT], wrist_flex, 5.0, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_MOTOR_JOINT], hand_motor, 5.0, &arm_joint_trajectory );
        checkPublishersConnection ( pub_arm_control_ );
        pub_arm_control_.publish( arm_joint_trajectory );
        ros::Duration( 5.0 ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}