#ifndef SOBIT_EDUCATION_CONTROLLER
#define SOBIT_EDUCATION_CONTROLLER

#include <sobit_education_library/sobit_turtlebot_controller.hpp>
#include <tf/transform_listener.h>
#include <cstring>
#include <cmath>

namespace sobit_education {
    enum Joint { 
                    ARM_ROLL_JOINT = 0, 
                    ARM_FLEX_JOINT, 
                    ELBOW_FLEX_JOINT, 
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
                                                            "arm_flex_joint", 
                                                            "elbow_flex_joint", 
                                                            "wrist_flex_joint", 
                                                            "hand_motor_joint", 
                                                            "xtion_pan_joint", 
                                                            "xtion_tilt_joint"
                                                        };

            static const double base_to_shoulder_flex_joint_z_cm = 52.2;
            static const double base_to_shoulder_flex_joint_x_cm = 12.2;
            static const double arm1_link_x_cm = 14.8;
            static const double arm1_link_z_cm = 2.4;
            static const double arm2_link_x_cm = 15.0;
            static const double can_grasp_min_z_cm = 35.0;
            static const double can_grasp_max_z_cm = 80.0;

            std::vector<Pose> pose_list_;
            void loadPose();
            bool moveAllJoint( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor, const double xtion_pan, const double xtion_tilt, const double sec, bool is_sleep = true );
        public:
            SobitEducationController( const std::string &name );
            SobitEducationController( );
            bool moveJoint ( const Joint joint_num, const double rad, const double sec, bool is_sleep = true );
            bool moveXtionPanTilt ( const double pan_rad, const double tilt_rad, const double sec, bool is_sleep = true );  
            bool moveArm ( const double arm_roll, const double arm_flex, const double elbow_flex, const double wrist_flex, const double hand_motor );
            bool movePose( const std::string &pose_name );
            bool moveGripperToTarget( const std::string &target_name, const double diff_x=0, const double diff_y=0, const double diff_z=0 );
    };
}
#endif