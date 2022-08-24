#ifndef SOBIT_TURTLEBOT_CONTROLLER
#define SOBIT_TURTLEBOT_CONTROLLER

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <pybind11/pybind11.h>


class ROSCommonNode
{
  public:
    ROSCommonNode( const std::string &name ) {
        char* cstr = new char[name.size() + 1];
        std::strcpy(cstr, name.c_str()); 
        char **argv = &cstr;
        int argc = 0;
        delete[] cstr;
        ros::init( argc, argv, "sobit_turtlebot_controller_node");
    }
    ROSCommonNode( ) { }
};

namespace sobit_education {
    class SobitTurtlebotController  : private ROSCommonNode {
        protected :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_cmd_vel_;
            ros::Subscriber sub_odom_;
            nav_msgs::Odometry curt_odom_;
            
            void checkPublishersConnection( const ros::Publisher& pub );
            void callbackOdometry( const nav_msgs::OdometryConstPtr &odom_msg );
            double geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat );
            double rad2Deg( const double rad );
            double deg2Rad( const double deg );
        public :
            SobitTurtlebotController( const std::string &name );
            SobitTurtlebotController( );
            bool controlWheelLinearFixed( const double distance );
            bool controlWheelLinear( const double distance );
            bool controlWheelRotateRad( const double angle_rad );
            bool controlWheelRotateDeg( const double angle_deg );
    };
}

inline void sobit_education::SobitTurtlebotController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );
    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep();
        } catch ( const std::exception& ex ) { break; }
    }
    return; 
}

inline void sobit_education::SobitTurtlebotController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_odom_ = *odom_msg; }

inline double sobit_education::SobitTurtlebotController::geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat ) {
    tf::Quaternion quat;
    double roll, pitch, yaw;
    quaternionMsgToTF( geometry_quat, quat );
    quat.normalize();
    tf::Matrix3x3( quat ).getRPY( roll, pitch, yaw );
    return yaw;    
}

inline double sobit_education::SobitTurtlebotController::rad2Deg ( const double rad ) { return rad * 180.0 / M_PI; }

inline double sobit_education::SobitTurtlebotController::deg2Rad ( const double deg ) { return deg * M_PI / 180.0; }

#endif /* SOBIT_TURTLEBOT_CONTROLLER */