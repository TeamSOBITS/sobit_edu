#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <sobit_education_control/sobit_education_hw.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "sobit_education_control_node");
  SobitEducationControl sobit_edu_ctr;
  sobit_edu_ctr.openPort();
  sobit_edu_ctr.initializeDynamixel();
  sobit_edu_ctr.writeInitialJoint();
  controller_manager::ControllerManager cm(&sobit_edu_ctr, sobit_edu_ctr.nh_);
  // ros::Rate rate(1 / sobit_edu_ctr.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  // ros::MultiThreadedSpinner spinner(4);
  spinner.start();
  ros::Duration(3).sleep();

  while (ros::ok()) {
    ros::Time     now = sobit_edu_ctr.getTime();
    ros::Duration dt  = sobit_edu_ctr.getPeriod();

    sobit_edu_ctr.read(now, dt);
    cm.update(now, dt);
    sobit_edu_ctr.write(now, dt);
  }
  spinner.stop();
  // spinner.spin();

  return 0;
}