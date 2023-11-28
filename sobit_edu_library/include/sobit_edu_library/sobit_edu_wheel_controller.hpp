#ifndef SOBIT_EDU_WHEEL_CONTROLLER_H_
#define SOBIT_EDU_WHEEL_CONTROLLER_H_

#include <cmath>
#include <cstring>

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "sobit_edu_library/sobit_edu_library.h"


namespace sobit_edu {
    class SobitEduWheelController  : private ROSCommonNode {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            
            ros::Publisher pub_cmd_vel_;
            ros::Subscriber sub_odom_;

            nav_msgs::Odometry curt_odom_;
            
            void checkPublishersConnection( const ros::Publisher& pub );
            void callbackOdometry( const nav_msgs::OdometryConstPtr &odom_msg );
            double geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat );
            

        public :
            SobitEduWheelController( const std::string &name );
            SobitEduWheelController( );

            bool controlWheelLinear( const double distance );
            bool controlWheelRotateRad( const double angle_rad );
            bool controlWheelRotateDeg( const double angle_deg );

            double rad2Deg( const double rad );
            double deg2Rad( const double deg );

    };
}

inline void sobit_edu::SobitEduWheelController::checkPublishersConnection ( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );

    while ( pub.getNumSubscribers()	== 0 && ros::ok() ) {
        try { loop_rate.sleep();
        } catch ( const std::exception& ex ) { break; }
    }

    return; 
}

inline void sobit_edu::SobitEduWheelController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_odom_ = *odom_msg; }

inline double sobit_edu::SobitEduWheelController::geometryQuat2Yaw( const geometry_msgs::Quaternion& geometry_quat ) {
    // tf::Quaternion quat;
    tf2::Quaternion quat_tf;
    double roll, pitch, yaw;

    tf2::fromMsg(geometry_quat, quat_tf);
    quat_tf.normalize();
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    // quaternionMsgToTF( geometry_quat, quat );
    // quat.normalize();
    // tf::Matrix3x3( quat ).getRPY( roll, pitch, yaw );
    
    return yaw;  
}

inline double sobit_edu::SobitEduWheelController::rad2Deg ( const double rad ) { return rad * 180.0 / M_PI; }

inline double sobit_edu::SobitEduWheelController::deg2Rad ( const double deg ) { return deg * M_PI / 180.0; }

#endif /* SOBIT_EDU_WHEEL_CONTROLLER_H_ */