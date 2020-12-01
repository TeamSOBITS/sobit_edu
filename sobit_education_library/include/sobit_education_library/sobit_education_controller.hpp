#ifndef SOBIT_EDUCATION_CONTROLLER
#define SOBIT_EDUCATION_CONTROLLER

#include <sobit_education_library/sobit_turtlebot_controller.hpp>

namespace sobit {
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
            const std::vector<std::string> joint_names_ = { "arm_roll_joint", 
                                                            "arm_flex_joint", 
                                                            "elbow_flex_joint", 
                                                            "wrist_flex_joint", 
                                                            "hand_motor_joint", 
                                                            "xtion_pan_joint", 
                                                            "xtion_tilt_joint"
                                                        };
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
    };
}
#endif