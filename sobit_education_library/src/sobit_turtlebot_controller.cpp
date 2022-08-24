#include <sobit_education_library/sobit_turtlebot_controller.hpp>

using namespace sobit_education;

SobitTurtlebotController::SobitTurtlebotController ( const std::string &name ) : ROSCommonNode( name ), nh_(), pnh_("~") {
    sub_odom_ = nh_.subscribe( "/odom", 1, &SobitTurtlebotController::callbackOdometry, this );
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
}

SobitTurtlebotController::SobitTurtlebotController ( ) : ROSCommonNode( ), nh_(), pnh_("~") { 
    sub_odom_ = nh_.subscribe( "/odom", 1, &SobitTurtlebotController::callbackOdometry, this );
    pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
}

bool SobitTurtlebotController::controlWheelLinearFixed( const double distance ) {
    try {
        // Wait until odom is received
        while ( curt_odom_.pose.pose.position.x == 0 &
                curt_odom_.pose.pose.position.y == 0 &
                curt_odom_.pose.pose.position.z == 0 &
                curt_odom_.pose.pose.orientation.w == 0 ) {

            ros::spinOnce();
        }

        geometry_msgs::Twist output_vel;
        nav_msgs::Odometry init_odom = curt_odom_;
        double start_time = ros::Time::now().toSec();

        double current_distance = 0.0;
        double target_distance  = std::fabs( distance );

        // double current_angle = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
        // double target_angle  = geometryQuat2Yaw( init_odom.pose.pose.orientation );
        double current_angle = curt_odom_.pose.pose.orientation.w;
        double target_angle  = init_odom.pose.pose.orientation.w;
        double target_angle_rad_abs = std::fabs( target_angle );
        double target_angle_deg_abs = rad2Deg( target_angle_rad_abs );

        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;

        double velocity_linear_differential = Kp * distance;
        double velocity_ang_differential    = Kp * target_angle;

        int loop_cnt = 1;
        ros::Rate loop_rate(20);

        // Start PID control
        while ( current_distance < target_distance  ) {
            ros::spinOnce();
            double end_time = ros::Time::now().toSec();
            double elapsed_time = end_time - start_time;
            // double current_angle = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
            double current_angle = curt_odom_.pose.pose.orientation.w;
            double vel_linear = 0.0;
            double vel_angular = 0.0;

            // PID for motion
            if ( target_distance <= 0.1 ) {
                vel_linear = Kp * ( target_distance + 0.001 - current_distance ) 
                           - Kd * velocity_linear_differential
                           + Ki / 0.8                       * ( target_distance + 0.001 - current_distance ) * std::pow( elapsed_time, 2 );
            } else {
                vel_linear = Kp * ( target_distance + 0.001 - current_distance )
                           - Kd * velocity_linear_differential
                           + Ki / ( 8.0 / target_distance ) * ( target_distance + 0.001 - current_distance ) * std::pow( elapsed_time, 2 );
            } 
            output_vel.linear.x = ( distance > 0 ) ? vel_linear : -vel_linear;

            // PID for rotation
            if ( target_angle_deg_abs <= 30 ) {
                vel_angular = Kp * ( target_angle_rad_abs + 0.001 - current_angle )
                            - Kd * velocity_ang_differential
                            + Ki * ( target_angle_rad_abs + 0.001 - current_angle ) * std::pow( elapsed_time, 2 );
            } else {
                vel_angular = Kp * ( target_angle_rad_abs + 0.001 - current_angle )
                            - Kd * velocity_ang_differential
                            + Ki * ( target_angle_rad_abs + 0.001 - current_angle ) * std::pow( elapsed_time, 2 ) * 0.75 * 30 / target_angle_deg_abs;
            }
            output_vel.angular.z = ( target_angle > 0 ) ? vel_angular : - vel_angular;

            pub_cmd_vel_.publish( output_vel );


            velocity_linear_differential = vel_linear;
            double x_diif = curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x;
            double y_diif = curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y;
            current_distance = std::hypotf( x_diif, y_diif );

            velocity_ang_differential = vel_angular;
            // double curt_yaw = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
            double curt_yaw = curt_odom_.pose.pose.orientation.w;
            double pre_move_ang_rad = current_angle;

            if ( -0.00314 < curt_yaw - target_angle && curt_yaw - target_angle < 0 && 0 < target_angle ) continue;
            else if ( 0 < curt_yaw - target_angle && curt_yaw - target_angle < 0.00314 && target_angle < 0 ) continue;

            if ( curt_yaw - target_angle < 0 && 0 < target_angle ) current_angle = abs(curt_yaw - target_angle + deg2Rad(360 * loop_cnt));
            else if ( 0 < curt_yaw - target_angle && target_angle < 0 ) current_angle = abs(curt_yaw - target_angle - deg2Rad(360 * loop_cnt));
            else if ( 0 < target_angle ) current_angle = abs(curt_yaw - target_angle + deg2Rad(360 * (loop_cnt-1)));
            else current_angle = abs(curt_yaw - target_angle - deg2Rad(360 * (loop_cnt-1)));

            if ( rad2Deg(current_angle) < (rad2Deg(pre_move_ang_rad)-0.0314) ) {
                loop_cnt++;
                if ( 0 < target_angle ) current_angle = abs(curt_yaw - target_angle + deg2Rad(360 * (loop_cnt-1)));
                else current_angle = abs(curt_yaw - target_angle - deg2Rad(360 * (loop_cnt-1)));
            }


            // ROS_INFO("target_distance = %f\tcurrent_distance = %f", target_distance, current_distance );
            ROS_INFO( "x_diif = %f\tx2_diif = %f\ty_diif = %f\ty2_diif = %f", init_odom.pose.pose.position.x, curt_odom_.pose.pose.position.x, init_odom.pose.pose.position.y, curt_odom_.pose.pose.position.y );
            ROS_INFO( "target_distance = %f\tcurrent_distance = %f", target_distance, current_distance );
            ROS_INFO( "target_angle = %f\tcurrent_angle = %f", target_angle_rad_abs, std::fabs(current_angle) );
            
            loop_rate.sleep();
        }
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitTurtlebotController::controlWheelLinear( const double distance ) {
    try {
        double start_time = ros::Time::now().toSec();
        geometry_msgs::Twist output_vel;
        while ( curt_odom_.pose.pose.position.x == 0 &
                curt_odom_.pose.pose.position.y == 0 &
                curt_odom_.pose.pose.position.z == 0 &
                curt_odom_.pose.pose.orientation.w == 0 ) {

            ros::spinOnce();
        }
        nav_msgs::Odometry init_odom = curt_odom_;
        double moving_distance = 0.0;
        double target_distance = std::fabs( distance );
        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;
        double velocity_differential = Kp * distance;
        ros::Rate loop_rate(20);
        while ( moving_distance < target_distance  ) {
            ros::spinOnce();
            double end_time = ros::Time::now().toSec();
            double elapsed_time = end_time - start_time;
            double vel_linear = 0.0;
            // PID
            if ( target_distance <= 0.1 ) {
                vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / 0.8 * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
            } else {
                vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / ( 8.0 / target_distance ) * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
            } 
            output_vel.linear.x = ( distance > 0 ) ? vel_linear : -vel_linear;
            velocity_differential = vel_linear;
            pub_cmd_vel_.publish( output_vel );
            double x_diif = curt_odom_.pose.pose.position.x - init_odom.pose.pose.position.x;
            double y_diif = curt_odom_.pose.pose.position.y - init_odom.pose.pose.position.y;
            moving_distance = std::hypotf( x_diif, y_diif );
            // ROS_INFO("target_distance = %f\tmoving_distance = %f", target_distance, moving_distance );
            ROS_INFO("x_diif = %f\tx2_diif = %f\ty_diif = %f\ty2_diif = %f", init_odom.pose.pose.position.x, curt_odom_.pose.pose.position.x, init_odom.pose.pose.position.y, curt_odom_.pose.pose.position.y );
            loop_rate.sleep();
        }
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool SobitTurtlebotController::controlWheelRotateRad( const double angle_rad ) {
    try {
        while ( curt_odom_.pose.pose.position.x == 0 &
                curt_odom_.pose.pose.position.y == 0 &
                curt_odom_.pose.pose.position.z == 0 &
                curt_odom_.pose.pose.orientation.w == 0 ) {

            ros::spinOnce();
        }
        double start_time = ros::Time::now().toSec();
        geometry_msgs::Twist output_vel;
        double init_yaw = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
        double moving_angle_rad = 0.0;
        double abs_angle_rad =  std::fabs( angle_rad );
        double abs_angle_deg = rad2Deg( abs_angle_rad );
        double Kp = 0.1;
        double Ki = 0.4;
        double Kd = 0.8;
        double velocity_differential = Kp * angle_rad;
        int loop_cnt = 1;
        ros::Rate loop_rate(20);

        while ( moving_angle_rad < abs_angle_rad ) {
            ros::spinOnce();
            double end_time = ros::Time::now().toSec();
            double elapsed_time = end_time - start_time;
            double vel_angular = 0.0;
            // PID
            if ( abs_angle_deg <= 30 ) {
                vel_angular = Kp * ( abs_angle_rad + 0.001 - moving_angle_rad )
                            - Kd * velocity_differential
                            + Ki * ( abs_angle_rad + 0.001 - moving_angle_rad ) * std::pow( elapsed_time, 2 );
            } else {
                vel_angular = Kp * ( abs_angle_rad + 0.001 - moving_angle_rad )
                            - Kd * velocity_differential
                            + Ki * ( abs_angle_rad + 0.001 - moving_angle_rad ) * std::pow( elapsed_time, 2 ) * 0.75 * 30 / abs_angle_deg;
            }
            output_vel.angular.z = ( angle_rad > 0 ) ? vel_angular : - vel_angular;
            velocity_differential = vel_angular;
            pub_cmd_vel_.publish( output_vel );
            
            
            double curt_yaw = geometryQuat2Yaw( curt_odom_.pose.pose.orientation );
            double pre_move_ang_rad = moving_angle_rad;
            if ( -0.00314 < curt_yaw - init_yaw && curt_yaw - init_yaw < 0 && 0 < angle_rad ) continue;
            else if ( 0 < curt_yaw - init_yaw && curt_yaw - init_yaw < 0.00314 && angle_rad < 0 ) continue;

            if ( curt_yaw - init_yaw < 0 && 0 < angle_rad ) moving_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * loop_cnt));
            else if ( 0 < curt_yaw - init_yaw && angle_rad < 0 ) moving_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * loop_cnt));
            else if ( 0 < angle_rad ) moving_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
            else moving_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));

            if ( rad2Deg(moving_angle_rad) < (rad2Deg(pre_move_ang_rad)-0.0314) ) {
                loop_cnt++;
                if ( 0 < angle_rad ) moving_angle_rad = abs(curt_yaw - init_yaw + deg2Rad(360 * (loop_cnt-1)));
                else moving_angle_rad = abs(curt_yaw - init_yaw - deg2Rad(360 * (loop_cnt-1)));
            }
            
            ROS_INFO( "target_angle = %f\tmoving_angle = %f", abs_angle_rad, moving_angle_rad );
            
            loop_rate.sleep();
        }
        return true;
    } catch ( const std::exception& ex ) {
        ROS_ERROR( "%s", ex.what() );
        return false;
    }
}

bool SobitTurtlebotController::controlWheelRotateDeg( const double angle_deg ) {
    return controlWheelRotateRad( deg2Rad(angle_deg) );
}