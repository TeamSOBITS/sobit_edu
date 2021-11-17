#ifndef SOBIT_EDUCATION_CONTROLLER
#define SOBIT_EDUCATION_CONTROLLER

#include <sobit_education_library/sobit_turtlebot_controller.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

namespace sobit_education {
    enum Joint { 
                    ARM_ROLL_JOINT = 0, 
                    ARM_FLEX_JOINT_RGT, 
                    ARM_FLEX_JOINT_LFT, 
                    ELBOW_FLEX_JOINT_RGT, 
                    ELBOW_FLEX_JOINT_LFT, 
                    WRIST_FLEX_JOINT, 
                    HAND_MOTOR_JOINT, 
                    XTION_PAN_JOINT, 
                    XTION_TILT_JOINT, 
                    JOINT_NUM 
                };

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
                                                            "xtion_tilt_joint"
                                                        };
            std::vector<Pose> pose_list_;

            static const double base_to_shoulder_flex_joint_z_cm;
            static const double base_to_shoulder_flex_joint_x_cm;
            static const double arm1_link_x_cm;
            static const double arm1_link_z_cm;
            static const double arm2_link_x_cm;
            static const double can_grasp_min_z_cm;
            static const double can_grasp_max_z_cm;

            void loadPose();
            bool moveAllJoint( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor, const double xtion_pan, const double xtion_tilt, const double sec, bool is_sleep = true );
            
        public:
            SobitEducationController( const std::string &name );
            SobitEducationController( );
            bool moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep = true );
            bool moveXtionPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep = true );  
            bool moveArm ( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor );
            bool movePose( const std::string &pose_name );
            bool moveGripperToTarget( const std::string &target_name, const double diff_x, const double diff_y, const double diff_z );
            bool moveGripperToTargetXYZ( const double target_x, const double target_y, const double target_z, const double diff_x, const double diff_y, const double diff_z );
    };
}
#endif