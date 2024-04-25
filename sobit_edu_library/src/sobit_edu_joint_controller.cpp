#include "sobit_edu_library/sobit_edu_joint_controller.hpp"
#include "sobit_edu_library/sobit_edu_wheel_controller.hpp"


using namespace sobit_edu;

const double sobit_edu::SobitEduJointController::base_to_shoulder_flex_joint_z_cm = 52.2;
const double sobit_edu::SobitEduJointController::base_to_shoulder_flex_joint_x_cm = 12.2;
const double sobit_edu::SobitEduJointController::arm_upper_link_x_cm = 14.8;
const double sobit_edu::SobitEduJointController::arm_upper_link_z_cm = 2.4;
const double sobit_edu::SobitEduJointController::arm_outer_link_x_cm = 15.0;
const double sobit_edu::SobitEduJointController::grasp_min_z_cm = 35.0;
const double sobit_edu::SobitEduJointController::grasp_max_z_cm = 80.0;

SobitEduJointController::SobitEduJointController( const std::string &name ) : ROSCommonNode(name), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/arm_trajectory_controller/command", 1) ;
    pub_head_camera_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/head_camera_trajectory_controller/command", 1 );

    sub_curr_arm    = nh_.subscribe( "/current_state_array", 1, &SobitEduJointController::callbackCurrArm, this );

    loadPose();
}

SobitEduJointController::SobitEduJointController( ) : ROSCommonNode(), nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/arm_trajectory_controller/command", 1 );
    pub_head_camera_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/head_camera_trajectory_controller/command", 1 );

    sub_curr_arm    = nh_.subscribe( "/current_state_array", 1, &SobitEduJointController::callbackCurrArm, this );

    loadPose();
}

void SobitEduJointController::loadPose( ) {
    XmlRpc::XmlRpcValue pose_val;

    if ( !nh_.hasParam("/sobit_edu_pose") ) return; 
    nh_.getParam( "/sobit_edu_pose", pose_val );

    int pose_num = pose_val.size();
    pose_list_.clear();

    for ( int i = 0; i < pose_num; i++ ) {
        Pose                pose;
        std::vector<double> joint_val(joint_name_.size(), 0.0);

        pose.pose_name                              =   static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::ARM_SHOULDER_PAN_JOINT]    =   static_cast<double>(pose_val[i][joint_names_[ARM_SHOULDER_PAN_JOINT]]); 
        joint_val[Joint::ARM_SHOULDER_1_TILT_JOINT] =   static_cast<double>(pose_val[i][joint_names_[ARM_SHOULDER_1_TILT_JOINT]]); 
        joint_val[Joint::ARM_SHOULDER_2_TILT_JOINT] = - static_cast<double>(pose_val[i][joint_names_[ARM_SHOULDER_1_TILT_JOINT]]); 
        joint_val[Joint::ARM_ELBOW_1_TILT_JOINT]    =   static_cast<double>(pose_val[i][joint_names_[ARM_ELBOW_1_TILT_JOINT]]); 
        joint_val[Joint::ARM_ELBOW_2_TILT_JOINT]    = - static_cast<double>(pose_val[i][joint_names_[ARM_ELBOW_1_TILT_JOINT]]); 
        joint_val[Joint::ARM_WRIST_TILT_JOINT]      =   static_cast<double>(pose_val[i][joint_names_[ARM_WRIST_TILT_JOINT]]); 
        joint_val[Joint::HAND_JOINT]                =   static_cast<double>(pose_val[i][joint_names_[HAND_JOINT]]); 
        joint_val[Joint::HEAD_CAMERA_PAN_JOINT]     =   static_cast<double>(pose_val[i][joint_names_[HEAD_CAMERA_PAN_JOINT]]); 
        joint_val[Joint::HEAD_CAMERA_TILT_JOINT]    =   static_cast<double>(pose_val[i][joint_names_[HEAD_CAMERA_TILT_JOINT]]); 
        
        pose.joint_val                              =   joint_val;
        
        pose_list_.push_back( pose );
    }

    return;
}

bool SobitEduJointController::moveToPose( const std::string &pose_name, const double sec ) {
    bool                is_find = false;
    std::vector<double> joint_val;
    for ( auto& pose : pose_list_ ) {
        if ( pose_name != pose.pose_name ) continue;
        is_find   = true;
        joint_val = pose.joint_val;
        break;
    }
    if ( is_find ) {
        ROS_INFO( "I found a '%s'", pose_name.c_str() );
        return moveAllJoint( joint_val[Joint::ARM_SHOULDER_PAN_JOINT], 
                             joint_val[Joint::ARM_SHOULDER_1_TILT_JOINT], 
                             joint_val[Joint::ARM_ELBOW_1_TILT_JOINT], 
                             joint_val[Joint::ARM_WRIST_TILT_JOINT], 
                             joint_val[Joint::HAND_JOINT], 
                             joint_val[Joint::HEAD_CAMERA_PAN_JOINT], 
                             joint_val[Joint::HEAD_CAMERA_TILT_JOINT],
                             sec );
    } else {
        ROS_ERROR( "'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitEduJointController::moveAllJoint( const double arm_shoulder_pan, 
                                             const double arm_shoulder_tilt, 
                                             const double arm_elbow_tilt, 
                                             const double arm_wrist_tilt, 
                                             const double hand, 
                                             const double head_camera_pan, 
                                             const double head_camera_tilt, 
                                             const double sec, 
                                             bool         is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        trajectory_msgs::JointTrajectory head_camera_joint_trajectory;

        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_PAN_JOINT], arm_shoulder_pan, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT], arm_shoulder_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT], -arm_shoulder_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_1_TILT_JOINT], arm_elbow_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_2_TILT_JOINT], -arm_elbow_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT], arm_wrist_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_JOINT], -hand, sec, &arm_joint_trajectory );
        
        setJointTrajectory( joint_names_[Joint::HEAD_CAMERA_PAN_JOINT], head_camera_pan, sec, &head_camera_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_CAMERA_TILT_JOINT], head_camera_tilt, sec, &head_camera_joint_trajectory );
        
        checkPublishersConnection ( pub_arm_control_ );
        checkPublishersConnection ( pub_head_camera_control_ );
        pub_arm_control_.publish( arm_joint_trajectory );
        pub_head_camera_control_.publish( head_camera_joint_trajectory );
        
        if ( is_sleep ) ros::Duration( sec ).sleep();
        
        return true;

    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEduJointController::moveJoint(    const Joint joint_num, 
                                            const double rad, 
                                            const double sec, 
                                            bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        // ARM_SHOULDER_1_TILT_JOINT : joint_num = 1
        // ARM_SHOULDER_1_TILT_JOINT : joint_num = 2
        // ARM_ELBOW_1_TILT_JOINT    : joint_num = 3
        // ARM_ELBOW_1_TILT_JOINT    : joint_num = 4
        if ( joint_num == 1 || joint_num == 3 ) {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
            addJointTrajectory(joint_names_[joint_num + 1], -rad, sec, &joint_trajectory);
        
        } else if ( joint_num == 2 || joint_num == 4 ) {
            setJointTrajectory( joint_names_[joint_num], -rad, sec, &joint_trajectory );
            addJointTrajectory(joint_names_[joint_num - 1], rad, sec, &joint_trajectory);
        
        } else if ( joint_num == 6 ) {
            setJointTrajectory( joint_names_[joint_num], -rad, sec, &joint_trajectory );
        
        } else {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        }
        
        if ( joint_num < Joint::HEAD_CAMERA_PAN_JOINT ) {
            checkPublishersConnection ( pub_arm_control_ );
            pub_arm_control_.publish( joint_trajectory );
        } else {
            checkPublishersConnection ( pub_head_camera_control_ );
            pub_head_camera_control_.publish( joint_trajectory );
        }
        
        if ( is_sleep ) ros::Duration( sec ).sleep();
        
        return true;
    
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEduJointController::moveHeadPanTilt(  const double pan_rad, 
                                                const double tilt_rad, 
                                                const double sec, 
                                                bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        setJointTrajectory( joint_names_[Joint::HEAD_CAMERA_PAN_JOINT], pan_rad, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HEAD_CAMERA_TILT_JOINT], tilt_rad, sec, &joint_trajectory );
        
        checkPublishersConnection ( pub_head_camera_control_ );
        pub_head_camera_control_.publish( joint_trajectory );
        
        if ( is_sleep ) ros::Duration( sec ).sleep();
        
        return true;
    
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEduJointController::moveArm (     const double arm_shoulder_pan, 
                                            const double arm_shoulder_tilt, 
                                            const double arm_elbow_tilt, 
                                            const double arm_wrist_tilt, 
                                            const double hand, 
                                            const double sec, 
                                            bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;

        setJointTrajectory( joint_names_[Joint::ARM_SHOULDER_PAN_JOINT], arm_shoulder_pan, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_1_TILT_JOINT], arm_shoulder_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_SHOULDER_2_TILT_JOINT], -arm_shoulder_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_1_TILT_JOINT], arm_elbow_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_ELBOW_2_TILT_JOINT], -arm_elbow_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_WRIST_TILT_JOINT], arm_wrist_tilt, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_JOINT], -hand, sec, &arm_joint_trajectory );
        
        checkPublishersConnection ( pub_arm_control_ );
        pub_arm_control_.publish( arm_joint_trajectory );
        
        if ( is_sleep ) ros::Duration( sec ).sleep();
        
        return true;
    
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

// TODO: Forward Kinematics Method (check result)

bool SobitEduJointController::moveHandToTargetCoord(    const double goal_position_x, 
                                                        const double goal_position_y, 
                                                        const double goal_position_z, 
                                                        const double diff_goal_position_x, 
                                                        const double diff_goal_position_y, 
                                                        const double diff_goal_position_z ){
    sobit_edu::SobitEduWheelController wheel_ctrl;
    geometry_msgs::Point shift;

    // // Calculate goal_position_pos + difference(gap)
    const double base_to_goal_position_x = goal_position_x + shift.x + diff_goal_position_x;
    const double base_to_goal_position_y = goal_position_y + shift.y + diff_goal_position_y;
    const double base_to_goal_position_z = goal_position_z + shift.z + diff_goal_position_z;

    // Calculate angle between footbase_pos and the sifted goal_position_pos (XY平面)
    double tan_rad = std::atan2(base_to_goal_position_y, base_to_goal_position_x);

    // Change goal_position_pos units (m->cm)
    const double goal_position_pos_x_cm = base_to_goal_position_x * 100.;
    const double goal_position_pos_y_cm = base_to_goal_position_y * 100.;
    const double goal_position_pos_z_cm = base_to_goal_position_z * 100.;

    // Check if the object is graspable
    if ( goal_position_pos_z_cm > grasp_max_z_cm ) {
        std::cout << "The target is located too tall ("  << goal_position_pos_z_cm << ">80.0)" << std::endl;
        return false;

    } else if ( goal_position_pos_z_cm < grasp_min_z_cm ) {
        std::cout << "The target is located too low (" << goal_position_pos_z_cm << "<35.0) " << std::endl;
        return false;
    }

    double shoulder_roll_joint_rad = 0.0;
    double shoulder_flex_joint_rad = 0.0;
    double arm_elbow_tilt_joint_rad = 0.0;
    double arm_wrist_tilt_joint_rad = 0.0;
    double hand_rad = 0.0;
    double base_to_arm_wrist_tilt_joint_x_cm = 0.0;

    // Target is above arm_elbow_tilt_join
    if ( (base_to_shoulder_flex_joint_z_cm + arm_upper_link_z_cm) < goal_position_pos_z_cm ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is above arm_elbow_tilt_join" << std::endl;
        ROS_INFO( "Target (z:%f) is above arm_elbow_tilt_join", goal_position_pos_z_cm );

        // Caution: Calculating until arm_wrist_tilt_joint_x_cm (not target)
        double arm_elbow_tilt_joint_sin = (goal_position_pos_z_cm - (base_to_shoulder_flex_joint_z_cm + arm_upper_link_x_cm)) / arm_outer_link_x_cm;
        arm_elbow_tilt_joint_rad = std::asin(arm_elbow_tilt_joint_sin);
        arm_wrist_tilt_joint_rad = -arm_elbow_tilt_joint_rad;
        shoulder_flex_joint_rad = 0.0;

        base_to_arm_wrist_tilt_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm_upper_link_z_cm + arm_outer_link_x_cm * std::cos(arm_elbow_tilt_joint_rad);
    }

    // Target is below arm_elbow_tilt_join and above shoulder_flex_joint
    else if ( base_to_shoulder_flex_joint_z_cm <= goal_position_pos_z_cm && goal_position_pos_z_cm <= (base_to_shoulder_flex_joint_z_cm + arm_upper_link_x_cm) ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is below arm_elbow_tilt_join and above shoulder_flex_joint" << std::endl;
        ROS_INFO( "Target (z:%f) is below arm_elbow_tilt_join and above shoulder_flex_joint", goal_position_pos_z_cm );

        // Caution: Calculating until arm_wrist_tilt_joint_x_cm (not target)
        double arm_elbow_tilt_joint_sin = (base_to_shoulder_flex_joint_z_cm + arm_upper_link_x_cm - goal_position_pos_z_cm) / arm_outer_link_x_cm;
        arm_elbow_tilt_joint_rad = -std::asin(arm_elbow_tilt_joint_sin);
        arm_wrist_tilt_joint_rad = -arm_elbow_tilt_joint_rad;
        shoulder_flex_joint_rad = 0.0;

        base_to_arm_wrist_tilt_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm_upper_link_z_cm + arm_outer_link_x_cm * std::cos(arm_elbow_tilt_joint_rad);
    }

    // Target is below shoulder_flex_joint
    else if ( goal_position_pos_z_cm < base_to_shoulder_flex_joint_z_cm ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is below shoulder_flex_joint" << std::endl;
        ROS_INFO( "Target (z:%f) is below shoulder_flex_joint", goal_position_pos_z_cm );

        // Caution: Calculating until arm_wrist_tilt_joint_x_cm (not target)
        double arm_elbow_tilt_joint_cos = (base_to_shoulder_flex_joint_z_cm - arm_upper_link_z_cm - goal_position_pos_z_cm) / arm_outer_link_x_cm;
        arm_elbow_tilt_joint_rad = std::acos(arm_elbow_tilt_joint_cos);
        arm_wrist_tilt_joint_rad = std::asin(arm_elbow_tilt_joint_cos);
        shoulder_flex_joint_rad = -wheel_ctrl.deg2Rad(90.0);

        base_to_arm_wrist_tilt_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm_upper_link_z_cm + arm_outer_link_x_cm * std::cos(arm_elbow_tilt_joint_rad);
    }

    // Calculate wheel movement (diagonal)
    // - Rotate the robot
    const double rot_rad = std::atan2(goal_position_pos_y_cm, goal_position_pos_x_cm);
    // ROS_INFO( "rot_rad = %f(deg:%f)", rot_rad, SobitEduWheelController::rad2Deg(rot_rad) );
    wheel_ctrl.controlWheelRotateRad(rot_rad);
    ros::Duration(3.0).sleep();

    // - Move forward the robot
    const double linear_m = std::sqrt(std::pow(goal_position_pos_x_cm, 2) + std::pow(goal_position_pos_y_cm, 2)) / 100.0 - base_to_arm_wrist_tilt_joint_x_cm / 100.0;
    // double linear_m = std::sqrt(std::pow(transform_base_to_target.getOrigin().x(), 2) + std::pow(transform_base_to_target.getOrigin().y(), 2)) - base_to_arm_wrist_tilt_joint_x_cm / 100. + diff_goal_position_x;
    ROS_INFO( "linear_m = %f", linear_m );
    wheel_ctrl.controlWheelLinear(linear_m);
    ros::Duration(3.0).sleep();

    // // Calculate wheel movement (+-90->x_pos->-+90->y_pos) NEEDS CONFIRMATION
    // // - Rotate the robot
    // const double rot_deg = goal_position_pos_x_cm > 0.0 ? 90.0:-90.0;
    // ROS_INFO("rot_deg:%f)", rot_deg);
    // wheel_ctrl.controlWheelRotateDeg(rot_deg);
    // ros::Duration(3.0).sleep();

    // // - Move forward the robot
    // ROS_INFO("linear_m = %f", goal_position_pos_x_cm/100.0);
    // wheel_ctrl.controlWheelLinear(goal_position_pos_x_cm/100.0);
    // ros::Duration(3.0).sleep();

    // // - Rotate the robot
    // ROS_INFO("rot_deg:%f)", -rot_deg);
    // wheel_ctrl.controlWheelRotateDeg(-rot_deg);
    // ros::Duration(3.0).sleep();

    // // - Move forward the robot
    // ROS_INFO("linear_m = %f", goal_position_pos_y_cm/100.0);
    // wheel_ctrl.controlWheelLinear(goal_position_pos_y_cm/100.0);
    // ros::Duration(3.0).sleep();

    // - Move arm (OPEN)
    hand_rad = wheel_ctrl.deg2Rad(90.0);
    bool is_reached = moveArm(shoulder_roll_joint_rad, shoulder_flex_joint_rad, arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, hand_rad);
    double shoulder_roll_joint_deg = wheel_ctrl.rad2Deg (shoulder_roll_joint_rad);
    double shoulder_flex_joint_deg = wheel_ctrl.rad2Deg (shoulder_flex_joint_rad);
    double arm_elbow_tilt_joint_deg = wheel_ctrl.rad2Deg (arm_elbow_tilt_joint_rad); 
    double arm_wrist_tilt_joint_deg = wheel_ctrl.rad2Deg (arm_wrist_tilt_joint_rad); 
    double hand_deg = wheel_ctrl.rad2Deg (hand_rad);
    printf( "ARM RAD: %f\t%f\t%f\t%f\t%f\n", shoulder_roll_joint_rad, shoulder_flex_joint_rad, arm_elbow_tilt_joint_rad, arm_wrist_tilt_joint_rad, hand_rad );
    printf( "ARM DEG: %f\t%f\t%f\t%f\t%f\n", shoulder_roll_joint_deg, shoulder_flex_joint_deg, arm_elbow_tilt_joint_deg, arm_wrist_tilt_joint_deg, hand_deg );
    ROS_INFO( "goal_position_pos = (%f, %f, %f)", goal_position_pos_x_cm, goal_position_pos_y_cm, goal_position_pos_z_cm );
    // ROS_INFO("result_pos = (%f, %f, %f)", for_kinematics_x, for_kinematics_z, for_kinematics_z);
    // ros::Duration(2.0).sleep();

    return is_reached;
}

bool SobitEduJointController::moveHandToTargetTF(   const std::string &goal_position_name, 
                                                    const double diff_goal_position_x, 
                                                    const double diff_goal_position_y, 
                                                    const double diff_goal_position_z ) {
    sobit_edu::SobitEduWheelController wheel_ctrl;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Point shift;
    
    bool tf_flag = false;

    try {
        tfBuffer_.canTransform( "base_footprint", goal_position_name, ros::Time(0), ros::Duration(2.0) );
        transformStamped = tfBuffer_.lookupTransform( "base_footprint", goal_position_name, ros::Time(0));
        tf_flag = true;
    } catch ( tf2::TransformException ex ) {
        ROS_ERROR( "ERROR: %s", ex.what() );
        return false;
    }

    // // Send goal_position_pos
    // const double goal_position_x = transform_base_to_target.getOrigin().x();
    // const double goal_position_y = transform_base_to_target.getOrigin().y();
    // const double goal_position_z = transform_base_to_target.getOrigin().z();
    auto& tf_target_to_arm = transformStamped.transform.translation;

    bool is_reached = moveHandToTargetCoord(tf_target_to_arm.x, tf_target_to_arm.y, tf_target_to_arm.z, diff_goal_position_x, diff_goal_position_y, diff_goal_position_z);

    return is_reached;
}

bool SobitEduJointController::moveHandToPlaceCoord( const double goal_position_x, 
                                                    const double goal_position_y, 
                                                    const double goal_position_z, 
                                                    const double diff_goal_position_x, 
                                                    const double diff_goal_position_y, 
                                                    const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    // 作成中
    double target_z         = 0.;

    /** 目標値から0.1[m]程下げた位置までアームを移動 **/
    /**  ハンドに負荷がかかった場合はそこで停止する  **/
    while( -target_z < diff_goal_position_z ) {
        moveHandToTargetCoord( goal_position_x, goal_position_y, goal_position_z, 
                                  diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
        
        // ハンドのジョイントに負荷がかかった場合、そこで停止する
        if ( 500 < arm_wrist_tilt_current_ && arm_wrist_tilt_current_ < 1000 ) {
            break;
        }

        // 目標値からの差分を追加
        target_z -= 0.05;
    }

    return true;
}

bool SobitEduJointController::moveHandToPlaceTF(    const std::string& target_name, 
                                                    const double diff_goal_position_x, 
                                                    const double diff_goal_position_y, 
                                                    const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    geometry_msgs::TransformStamped transform_base_to_target;
    try {
        tfBuffer_.canTransform("base_footprint", target_name, ros::Time(0), ros::Duration(2.0));
        transform_base_to_target = tfBuffer_.lookupTransform("base_footprint", target_name, ros::Time(0));
    } catch ( tf2::TransformException ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    auto& goal_position = transform_base_to_target.transform.translation;

    moveHandToPlaceCoord( goal_position.x, goal_position.y, goal_position.z,
                                        diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
    return true;
}

bool SobitEduJointController::graspDecision( const int min_curr, const int max_curr ) {
    bool is_grasped = false;

    // Spin until the current value is obtained
    while( hand_current_ == 0. ) ros::spinOnce();

    // ros::spinOnce();
    std::cout << "hand_current_ = " << hand_current_ << std::endl;

    is_grasped = (min_curr <= hand_current_ && hand_current_ <= max_curr) ? true : false;

    return is_grasped;
}

bool SobitEduJointController::placeDecision( const int min_curr, const int max_curr ){
    bool is_placed = false;

    // Spin until the current value is obtained
    while( arm_wrist_tilt_current_ == 0. ) ros::spinOnce();

    // ros::spinOnce();
    std::cout << "arm_wrist_tilt_joint_curr_ = " << arm_wrist_tilt_current_ << std::endl;

    is_placed = (min_curr <= arm_wrist_tilt_current_ && arm_wrist_tilt_current_ <= max_curr) ? true : false;

    return is_placed;
}