#include <sobit_education_library/sobit_education_controller.hpp>

using namespace sobit_education;

const double sobit_education::SobitEducationController::base_to_shoulder_flex_joint_z_cm = 52.2;
const double sobit_education::SobitEducationController::base_to_shoulder_flex_joint_x_cm = 12.2;
const double sobit_education::SobitEducationController::arm1_link_x_cm = 14.8;
const double sobit_education::SobitEducationController::arm1_link_z_cm = 2.4;
const double sobit_education::SobitEducationController::arm2_link_x_cm = 15.0;
const double sobit_education::SobitEducationController::grasp_min_z_cm = 35.0;
const double sobit_education::SobitEducationController::grasp_max_z_cm = 80.0;

SobitEducationController::SobitEducationController( const std::string &name ) : SobitTurtlebotController( name ) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/arm_trajectory_controller/command", 1) ;
    pub_xtion_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/xtion_trajectory_controller/command", 1 );
    loadPose();
}

SobitEducationController::SobitEducationController( ) : SobitTurtlebotController( ) {
    pub_arm_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/arm_trajectory_controller/command", 1 );
    pub_xtion_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>( "/xtion_trajectory_controller/command", 1 );    
    loadPose();
}

void SobitEducationController::loadPose( ) {
    XmlRpc::XmlRpcValue pose_val;
    if ( !nh_.hasParam("/education_pose") ) return; 
    nh_.getParam( "/education_pose", pose_val );
    int pose_num = pose_val.size();
    pose_list_.clear();
    for ( int i = 0; i < pose_num; i++ ) {
        Pose                pose;
        std::vector<double> joint_val(9, 0.0);
        pose.pose_name                         = static_cast<std::string>(pose_val[i]["pose_name"]); 
        joint_val[Joint::ARM_ROLL_JOINT]       = static_cast<double>(pose_val[i][joint_names_[ARM_ROLL_JOINT]]); 
        joint_val[Joint::ARM_FLEX_JOINT_RGT]   = static_cast<double>(pose_val[i][joint_names_[ARM_FLEX_JOINT_RGT]]); 
        joint_val[Joint::ARM_FLEX_JOINT_LFT]   = static_cast<double>(pose_val[i][joint_names_[ARM_FLEX_JOINT_LFT]]); 
        joint_val[Joint::ELBOW_FLEX_JOINT_RGT] = static_cast<double>(pose_val[i][joint_names_[ELBOW_FLEX_JOINT_RGT]]); 
        joint_val[Joint::ELBOW_FLEX_JOINT_LFT] = static_cast<double>(pose_val[i][joint_names_[ELBOW_FLEX_JOINT_LFT]]); 
        joint_val[Joint::WRIST_FLEX_JOINT]     = static_cast<double>(pose_val[i][joint_names_[WRIST_FLEX_JOINT]]); 
        joint_val[Joint::HAND_MOTOR_JOINT]     = static_cast<double>(pose_val[i][joint_names_[HAND_MOTOR_JOINT]]); 
        joint_val[Joint::XTION_PAN_JOINT]      = static_cast<double>(pose_val[i][joint_names_[XTION_PAN_JOINT]]); 
        joint_val[Joint::XTION_TILT_JOINT]     = static_cast<double>(pose_val[i][joint_names_[XTION_TILT_JOINT]]); 
        pose.joint_val                         = joint_val;
        pose_list_.push_back( pose );
    }
    return;
}

bool SobitEducationController::moveToPose( const std::string &pose_name, const double sec ) {
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
        return moveAllJoint( joint_val[Joint::ARM_ROLL_JOINT], 
                             joint_val[Joint::ARM_FLEX_JOINT_RGT], 
                             joint_val[Joint::ELBOW_FLEX_JOINT_RGT], 
                             joint_val[Joint::WRIST_FLEX_JOINT], 
                             joint_val[Joint::HAND_MOTOR_JOINT], 
                             joint_val[Joint::XTION_PAN_JOINT], 
                             joint_val[Joint::XTION_TILT_JOINT],
                             sec );
    } else {
        ROS_ERROR( "'%s' doesn't exist.", pose_name.c_str() );
        return false;
    } 
}

bool SobitEducationController::moveAllJoint( const double arm_roll, 
                                             const double arm_flex, 
                                             const double elbow_flex, 
                                             const double wrist_flex, 
                                             const double hand_motor, 
                                             const double xtion_pan, 
                                             const double xtion_tilt, 
                                             const double sec, 
                                             bool         is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        trajectory_msgs::JointTrajectory xtion_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::ARM_ROLL_JOINT], arm_roll, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT_RGT], arm_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT_LFT], -arm_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT_RGT], elbow_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT_LFT], -elbow_flex, sec, &arm_joint_trajectory );
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
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEducationController::moveJoint( const Joint joint_num, const double rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;

        // ARM_FLEX_JOINT_RGT   : joint_num = 1
        // ARM_FLEX_JOINT_RGT   : joint_num = 2
        // ELBOW_FLEX_JOINT_RGT : joint_num = 3
        // ELBOW_FLEX_JOINT_RGT : joint_num = 4
        if ( joint_num == 1 || joint_num == 3 ) {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
            addJointTrajectory(joint_names_[joint_num + 1], -rad, sec, &joint_trajectory);
        } else if ( joint_num == 2 || joint_num == 4 ) {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
            addJointTrajectory(joint_names_[joint_num - 1], -rad, sec, &joint_trajectory);
        } else {
            setJointTrajectory( joint_names_[joint_num], rad, sec, &joint_trajectory );
        }
        
        if ( joint_num < Joint::XTION_PAN_JOINT ) {
            checkPublishersConnection ( pub_arm_control_ );
            pub_arm_control_.publish( joint_trajectory );
        } else {
            checkPublishersConnection ( pub_xtion_control_ );
            pub_xtion_control_.publish( joint_trajectory );
        }
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEducationController::moveHeadPanTilt( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory joint_trajectory;
        setJointTrajectory( joint_names_[Joint::XTION_PAN_JOINT], pan_rad, sec, &joint_trajectory );
        addJointTrajectory( joint_names_[Joint::XTION_TILT_JOINT], tilt_rad, sec, &joint_trajectory );
        checkPublishersConnection ( pub_xtion_control_ );
        pub_xtion_control_.publish( joint_trajectory );
        if ( is_sleep ) ros::Duration( sec ).sleep();
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitEducationController::moveArm ( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor, const double sec, bool is_sleep ) {
    try {
        trajectory_msgs::JointTrajectory arm_joint_trajectory;
        setJointTrajectory( joint_names_[Joint::ARM_ROLL_JOINT], arm_roll, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT_RGT], arm_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ARM_FLEX_JOINT_LFT], -arm_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT_RGT], elbow_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::ELBOW_FLEX_JOINT_LFT], -elbow_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::WRIST_FLEX_JOINT], wrist_flex, sec, &arm_joint_trajectory );
        addJointTrajectory( joint_names_[Joint::HAND_MOTOR_JOINT], hand_motor, sec, &arm_joint_trajectory );
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

bool SobitEducationController::moveGripperToTargetCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ){
    sobit_education::SobitTurtlebotController wheel_ctrl;
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
    double elbow_flex_joint_rad = 0.0;
    double wrist_flex_joint_rad = 0.0;
    double hand_rad = 0.0;
    double base_to_wrist_flex_joint_x_cm = 0.0;

    // Target is above elbow_flex_join
    if ( (base_to_shoulder_flex_joint_z_cm + arm1_link_z_cm) < goal_position_pos_z_cm ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is above elbow_flex_join" << std::endl;
        ROS_INFO( "Target (z:%f) is above elbow_flex_join", goal_position_pos_z_cm );

        // Caution: Calculating until wrist_flex_joint_x_cm (not target)
        double elbow_flex_joint_sin = (goal_position_pos_z_cm - (base_to_shoulder_flex_joint_z_cm + arm1_link_x_cm)) / arm2_link_x_cm;
        elbow_flex_joint_rad = std::asin(elbow_flex_joint_sin);
        wrist_flex_joint_rad = -elbow_flex_joint_rad;
        shoulder_flex_joint_rad = 0.0;

        base_to_wrist_flex_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm1_link_z_cm + arm2_link_x_cm * std::cos(elbow_flex_joint_rad);
    }

    // Target is below elbow_flex_join and above shoulder_flex_joint
    else if ( base_to_shoulder_flex_joint_z_cm <= goal_position_pos_z_cm && goal_position_pos_z_cm <= (base_to_shoulder_flex_joint_z_cm + arm1_link_x_cm) ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is below elbow_flex_join and above shoulder_flex_joint" << std::endl;
        ROS_INFO( "Target (z:%f) is below elbow_flex_join and above shoulder_flex_joint", goal_position_pos_z_cm );

        // Caution: Calculating until wrist_flex_joint_x_cm (not target)
        double elbow_flex_joint_sin = (base_to_shoulder_flex_joint_z_cm + arm1_link_x_cm - goal_position_pos_z_cm) / arm2_link_x_cm;
        elbow_flex_joint_rad = -std::asin(elbow_flex_joint_sin);
        wrist_flex_joint_rad = -elbow_flex_joint_rad;
        shoulder_flex_joint_rad = 0.0;

        base_to_wrist_flex_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm1_link_z_cm + arm2_link_x_cm * std::cos(elbow_flex_joint_rad);
    }

    // Target is below shoulder_flex_joint
    else if ( goal_position_pos_z_cm < base_to_shoulder_flex_joint_z_cm ) {
        std::cout << "Target (z:" << goal_position_pos_z_cm << ") is below shoulder_flex_joint" << std::endl;
        ROS_INFO( "Target (z:%f) is below shoulder_flex_joint", goal_position_pos_z_cm );

        // Caution: Calculating until wrist_flex_joint_x_cm (not target)
        double elbow_flex_joint_cos = (base_to_shoulder_flex_joint_z_cm - arm1_link_z_cm - goal_position_pos_z_cm) / arm2_link_x_cm;
        elbow_flex_joint_rad = std::acos(elbow_flex_joint_cos);
        wrist_flex_joint_rad = std::asin(elbow_flex_joint_cos);
        shoulder_flex_joint_rad = -SobitTurtlebotController::deg2Rad(90.0);

        base_to_wrist_flex_joint_x_cm = base_to_shoulder_flex_joint_x_cm + arm1_link_z_cm + arm2_link_x_cm * std::cos(elbow_flex_joint_rad);
    }

    // Calculate wheel movement (diagonal)
    // - Rotate the robot
    const double rot_rad = std::atan2(goal_position_pos_y_cm, goal_position_pos_x_cm);
    ROS_INFO( "rot_rad = %f(deg:%f)", rot_rad, SobitTurtlebotController::rad2Deg(rot_rad) );
    wheel_ctrl.controlWheelRotateRad(rot_rad);
    ros::Duration(3.0).sleep();

    // - Move forward the robot
    const double linear_m = std::sqrt(std::pow(goal_position_pos_x_cm, 2) + std::pow(goal_position_pos_y_cm, 2)) / 100.0 - base_to_wrist_flex_joint_x_cm / 100.0;
    // double linear_m = std::sqrt(std::pow(transform_base_to_target.getOrigin().x(), 2) + std::pow(transform_base_to_target.getOrigin().y(), 2)) - base_to_wrist_flex_joint_x_cm / 100. + diff_goal_position_x;
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
    hand_rad = SobitTurtlebotController::deg2Rad(90.0);
    bool is_reached = moveArm(shoulder_roll_joint_rad, shoulder_flex_joint_rad, elbow_flex_joint_rad, wrist_flex_joint_rad, hand_rad);
    printf( "ARM RAD: %f\t%f\t%f\t%f\t%f\n", shoulder_roll_joint_rad, shoulder_flex_joint_rad, elbow_flex_joint_rad, wrist_flex_joint_rad, hand_rad );
    printf( "ARM DEG: %f\t%f\t%f\t%f\t%f\n", rad2Deg (shoulder_roll_joint_rad), rad2Deg (shoulder_flex_joint_rad), rad2Deg (elbow_flex_joint_rad), rad2Deg (wrist_flex_joint_rad), rad2Deg (hand_rad) );
    ROS_INFO( "goal_position_pos = (%f, %f, %f)", goal_position_pos_x_cm, goal_position_pos_y_cm, goal_position_pos_z_cm );
    // ROS_INFO("result_pos = (%f, %f, %f)", for_kinematics_x, for_kinematics_z, for_kinematics_z);
    ros::Duration(2.0).sleep();

    return is_reached;
}

bool SobitEducationController::moveGripperToTargetTF( const std::string &goal_position_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    sobit_education::SobitTurtlebotController wheel_ctrl;
    tf::StampedTransform transform_base_to_target;
    geometry_msgs::Point shift;
    
    bool tf_flag = false;

    try {
        listener_.waitForTransform( "base_footprint", goal_position_name, ros::Time(0), ros::Duration(2.0) );
        listener_.lookupTransform( "base_footprint", goal_position_name, ros::Time(0), transform_base_to_target );
        tf_flag = true;
    } catch ( tf::TransformException ex ) {
        ROS_ERROR( "ERROR: %s", ex.what() );
        return false;
    }

    // // Send goal_position_pos
    const double goal_position_x = transform_base_to_target.getOrigin().x();
    const double goal_position_y = transform_base_to_target.getOrigin().y();
    const double goal_position_z = transform_base_to_target.getOrigin().z();

    bool is_reached = moveGripperToTargetCoord(goal_position_x, goal_position_y, goal_position_z, diff_goal_position_x, diff_goal_position_y, diff_goal_position_z);

    return is_reached;
}

bool SobitEducationController::moveGripperToPlaceCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    // 作成中
    double target_z         = 0.;

    /** 目標値から0.1[m]程下げた位置までアームを移動 **/
    /**  ハンドに負荷がかかった場合はそこで停止する  **/
    while( -target_z < diff_goal_position_z ) {
        moveGripperToTargetCoord( goal_position_x, goal_position_y, goal_position_z, 
                                  diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
        
        // ハンドのジョイントに負荷がかかった場合、そこで停止する
        if ( 500 < wrist_flex_current_ && wrist_flex_current_ < 1000 ) {
            break;
        }

        // 目標値からの差分を追加
        target_z -= 0.05;
    }

    return true;
}

bool SobitEducationController::moveGripperToPlaceTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z ) {
    geometry_msgs::Point shift;

    tf::StampedTransform transform_base_to_target;
    try {
        listener_.waitForTransform("base_footprint", target_name, ros::Time(0), ros::Duration(2.0));
        listener_.lookupTransform("base_footprint", target_name, ros::Time(0), transform_base_to_target);
    } catch ( tf::TransformException ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    moveGripperToPlaceCoord( transform_base_to_target.getOrigin().x(), transform_base_to_target.getOrigin().y(), transform_base_to_target.getOrigin().z(),
                                        diff_goal_position_x, diff_goal_position_y, diff_goal_position_z );
    return true;
}

bool SobitEducationController::graspDecision() {
    while ( hand_motor_current_ == 0. ) {
        ros::spinOnce();
    }
    ros::spinOnce();
    std::cout << "hand_motor_current_ :" << hand_motor_current_ << std::endl;
    if ( 500 <= hand_motor_current_ && hand_motor_current_ <= 1000 ) {
        return true;
    } else {
        return false;
    }
}

void SobitEducationController::callbackCurrentStateArray( const sobit_common_msg::current_state_array msg ) {
    ros::spinOnce();

    for ( const auto current_state : msg.current_state_array ) {
        if ( current_state.joint_name == "WRIST_FLEX_JOINT" ) {
            //std::cout << "\njoint_name:" << current_state.joint_name << std::endl;
            //std::cout << "\njoint_current:" << current_state.current_ma << std::endl;
            wrist_flex_current_ = current_state.current_ma;
        }

        if ( current_state.joint_name == "HAND_MOTOR_JOINT" ) {
            //std::cout << "\njoint_name:" << current_state.joint_name << std::endl;
            //std::cout << "\njoint_current:" << current_state.current_ma << std::endl;
            hand_motor_current_ = current_state.current_ma;
        }
    }
}