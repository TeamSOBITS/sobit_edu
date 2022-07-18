#ifndef SOBIT_EDUCATION_CONTROLLER
#define SOBIT_EDUCATION_CONTROLLER

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <sobit_education_library/sobit_turtlebot_controller.hpp>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sobit_common_msg/current_state.h>
#include <sobit_common_msg/current_state_array.h>

namespace sobit_education {
    enum Joint { ARM_ROLL_JOINT = 0, 
                ARM_FLEX_JOINT_RGT, 
                ARM_FLEX_JOINT_LFT, 
                ELBOW_FLEX_JOINT_RGT, 
                ELBOW_FLEX_JOINT_LFT, 
                WRIST_FLEX_JOINT, 
                HAND_MOTOR_JOINT, 
                XTION_PAN_JOINT, 
                XTION_TILT_JOINT, 
                JOINT_NUM };

    typedef struct {         
        std::string pose_name;
        std::vector<double> joint_val;
    } Pose;

    class SobitEducationController : public SobitTurtlebotController {
        private:
            ros::Publisher pub_arm_control_;
            ros::Publisher pub_xtion_control_;  
            tf::TransformListener listener_;

            const std::vector<std::string> joint_names_ = { "arm_roll_joint", 
                                                            "arm_flex_joint_rgt", 
                                                            "arm_flex_joint_lft", 
                                                            "elbow_flex_joint_rgt", 
                                                            "elbow_flex_joint_lft", 
                                                            "wrist_flex_joint", 
                                                            "hand_motor_joint", 
                                                            "xtion_pan_joint", 
                                                            "xtion_tilt_joint" };
            std::vector<Pose> pose_list_;

            static const double base_to_shoulder_flex_joint_z_cm;
            static const double base_to_shoulder_flex_joint_x_cm;
            static const double arm1_link_x_cm;
            static const double arm1_link_z_cm;
            static const double arm2_link_x_cm;
            static const double grasp_min_z_cm;
            static const double grasp_max_z_cm;

            void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void checkPublishersConnection( const ros::Publisher& pub );
            void loadPose( );
            bool moveAllJoint( const double arm_roll,
                                const double arm_flex,
                                const double elbow_flex,
                                const double wrist_flex,
                                const double hand_motor,
                                const double xtion_pan,
                                const double xtion_tilt,
                                const double sec,
                                bool         is_sleep = true );

        double wrist_flex_current_ = 0.;
        double hand_motor_current_ = 0.;
        void callbackCurrentStateArray( const sobit_common_msg::current_state_array );
        ros::Subscriber sub_current_state_array = nh_.subscribe( "/current_state_array", 1, &SobitEducationController::callbackCurrentStateArray, this );

        public:
            SobitEducationController( const std::string &name );
            SobitEducationController( );
            
            bool moveToPose( const std::string &pose_name, const double sec = 5.0 );
            bool moveJoint( const Joint joint_num, const double rad, const double sec = 5.0, bool is_sleep = true );
            bool moveHeadPanTilt( const double pan_rad, const double tilt_rad, const double sec = 5.0, bool is_sleep = true );  
            bool moveArm( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToTargetCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToTargetTF( const std::string &target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToPlaceCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToPlaceTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool graspDecision( );
    };
} // namespace sobit_education

inline void sobit_education::SobitEducationController::setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point; 
    joint_trajectory.joint_names.push_back( joint_name ); 
    joint_trajectory_point.positions.push_back( rad );
    joint_trajectory_point.velocities.push_back( 0.0 );
    joint_trajectory_point.accelerations.push_back( 0.0 );
    joint_trajectory_point.effort.push_back( 0.0 );
    joint_trajectory_point.time_from_start = ros::Duration( sec );
    joint_trajectory.points.push_back( joint_trajectory_point );
    *jt = joint_trajectory;
    return;
}

inline void sobit_education::SobitEducationController::addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory = *jt;
    joint_trajectory.joint_names.push_back( joint_name ); 
    joint_trajectory.points[0].positions.push_back( rad );
    joint_trajectory.points[0].velocities.push_back( 0.0 );
    joint_trajectory.points[0].accelerations.push_back( 0.0 );
    joint_trajectory.points[0].effort.push_back( 0.0 );
    joint_trajectory.points[0].time_from_start = ros::Duration( sec );
    *jt = joint_trajectory;
    return;
}

inline void sobit_education::SobitEducationController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep();
        } catch ( const std::exception& ex ) { break; }
    }
    return; 
}

#endif /* SOBIT_EDUCATION_CONTROLLER */