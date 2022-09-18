#ifndef SOBIT_EDU_CONTROLLER
#define SOBIT_EDU_CONTROLLER

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <sobit_edu_library/sobit_turtlebot_controller.hpp>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sobit_common_msg/current_state.h>
#include <sobit_common_msg/current_state_array.h>

namespace sobit_edu {
    enum Joint { ARM_SHOULDER_PAN_JOINT = 0, 
                 ARM_SHOULDER_1_TILT_JOINT, 
                 ARM_SHOULDER_2_TILT_JOINT, 
                 ARM_ELBOW_1_TILT_JOINT, 
                 ARM_ELBOW_2_TILT_JOINT, 
                 ARM_WRIST_TILT_JOINT, 
                 HAND_JOINT, 
                 HEAD_CAMERA_PAN_JOINT, 
                 HEAD_CAMERA_TILT_JOINT, 
                 JOINT_NUM };

    typedef struct {         
        std::string pose_name;
        std::vector<double> joint_val;
    } Pose;

    class SobitEduController : public SobitTurtlebotController {
        private:
            ros::Publisher pub_arm_control_;
            ros::Publisher pub_head_camera_control_;  
            tf::TransformListener listener_;

            const std::vector<std::string> joint_names_ = { "arm_shoulder_pan_joint", 
                                                            "arm_shoulder_1_tilt_joint", 
                                                            "arm_shoulder_2_tilt_joint", 
                                                            "arm_elbow_1_tilt_joint", 
                                                            "arm_elbow_2_tilt_joint", 
                                                            "arm_wrist_tilt_joint", 
                                                            "hand_joint", 
                                                            "head_camera_pan_joint", 
                                                            "head_camera_tilt_joint" };
            std::vector<Pose> pose_list_;

            static const double base_to_shoulder_flex_joint_z_cm;
            static const double base_to_shoulder_flex_joint_x_cm;
            static const double arm_upper_link_x_cm;
            static const double arm_upper_link_z_cm;
            static const double arm_outer_link_x_cm;
            static const double grasp_min_z_cm;
            static const double grasp_max_z_cm;

            void setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt );
            void checkPublishersConnection( const ros::Publisher& pub );
            void loadPose( );
            bool moveAllJoint( const double arm_shoulder_pan,
                               const double arm_shoulder_tilt,
                               const double arm_elbow_tilt,
                               const double arm_wrist_tilt,
                               const double hand,
                               const double head_camera_pan,
                               const double head_camera_tilt,
                               const double sec,
                               bool         is_sleep = true );

            double arm_wrist_tilt_current_ = 0.;
            double hand_current_ = 0.;
            void callbackCurrentStateArray( const sobit_common_msg::current_state_array );
            ros::Subscriber sub_current_state_array = nh_.subscribe( "/current_state_array", 1, &SobitEduController::callbackCurrentStateArray, this );

        public:
            SobitEduController( const std::string &name );
            SobitEduController( );
            
            bool moveToPose( const std::string &pose_name, const double sec = 5.0 );
            bool moveJoint( const Joint joint_num, const double rad, const double sec = 5.0, bool is_sleep = true );
            bool moveHeadPanTilt( const double pan_rad, const double tilt_rad, const double sec = 5.0, bool is_sleep = true );  
            bool moveArm( const double arm_shoulder_pan, const double arm_shoulder_tilt, const double arm_elbow_tilt, const double arm_wrist_tilt, const double hand, const double sec = 5.0, bool is_sleep = true );
            bool moveGripperToTargetCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToTargetTF( const std::string &target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToPlaceCoord( const double goal_position_x, const double goal_position_y, const double goal_position_z, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool moveGripperToPlaceTF( const std::string& target_name, const double diff_goal_position_x, const double diff_goal_position_y, const double diff_goal_position_z );
            bool graspDecision( );
    };
} // namespace sobit_edu

inline void sobit_edu::SobitEduController::setJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
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

inline void sobit_edu::SobitEduController::addJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ) {
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

inline void sobit_edu::SobitEduController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep();
        } catch ( const std::exception& ex ) { break; }
    }
    return; 
}

#endif /* SOBIT_EDU_CONTROLLER */