#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <sobit_education_control/sobit_education_hw.h>
#include <trajectory_msgs/JointTrajectory.h>

SobitEducationControl::SobitEducationControl() {
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_arm_roll("arm_roll_joint", &pos_[0], &vel_[0], &eff_[0]);
  hardware_interface::JointStateHandle state_handle_arm_flex("arm_flex_joint", &pos_[1], &vel_[1], &eff_[1]);
  hardware_interface::JointStateHandle state_handle_elbow_flex("elbow_flex_joint", &pos_[2], &vel_[2], &eff_[2]);
  hardware_interface::JointStateHandle state_handle_wrist_flex("wrist_flex_joint", &pos_[3], &vel_[3], &eff_[3]);
  hardware_interface::JointStateHandle state_handle_hand_motor("hand_motor_joint", &pos_[4], &vel_[4], &eff_[4]);
  hardware_interface::JointStateHandle state_handle_xtion_tilt("xtion_tilt_joint", &pos_[5], &vel_[5], &eff_[5]);
  jnt_state_interface_.registerHandle(state_handle_arm_roll);
  jnt_state_interface_.registerHandle(state_handle_arm_flex);
  jnt_state_interface_.registerHandle(state_handle_elbow_flex);
  jnt_state_interface_.registerHandle(state_handle_wrist_flex);
  jnt_state_interface_.registerHandle(state_handle_hand_motor);
  jnt_state_interface_.registerHandle(state_handle_xtion_tilt);
  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_arm_roll(jnt_state_interface_.getHandle("arm_roll_joint"), &cmd_[0]);
  hardware_interface::JointHandle pos_handle_arm_flex(jnt_state_interface_.getHandle("arm_flex_joint"), &cmd_[1]);
  hardware_interface::JointHandle pos_handle_elbow_flex(jnt_state_interface_.getHandle("elbow_flex_joint"), &cmd_[2]);
  hardware_interface::JointHandle pos_handle_wrist_flex(jnt_state_interface_.getHandle("wrist_flex_joint"), &cmd_[3]);
  hardware_interface::JointHandle pos_handle_hand_motor(jnt_state_interface_.getHandle("hand_motor_joint"), &cmd_[4]);
  hardware_interface::JointHandle pos_handle_xtion_tilt(jnt_state_interface_.getHandle("xtion_tilt_joint"), &cmd_[5]);
  jnt_pos_interface_.registerHandle(pos_handle_arm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_arm_flex);
  jnt_pos_interface_.registerHandle(pos_handle_elbow_flex);
  jnt_pos_interface_.registerHandle(pos_handle_wrist_flex);
  jnt_pos_interface_.registerHandle(pos_handle_hand_motor);
  jnt_pos_interface_.registerHandle(pos_handle_xtion_tilt);
  registerInterface(&jnt_pos_interface_);

  // register the joint limits interface
  joint_limits_interface::JointLimits     joint_limits;
  joint_limits_interface::SoftJointLimits soft_joint_limits;
  getJointLimits("arm_roll_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_arm_roll(
      pos_handle_arm_roll, joint_limits, soft_joint_limits);
  getJointLimits("arm_flex_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_arm_flex(
      pos_handle_arm_flex, joint_limits, soft_joint_limits);
  getJointLimits("elbow_flex_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_elbow_flex(
      pos_handle_elbow_flex, joint_limits, soft_joint_limits);
  getJointLimits("wrist_flex_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_wrist_flex(
      pos_handle_wrist_flex, joint_limits, soft_joint_limits);
  getJointLimits("hand_motor_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_hand_motor(
      pos_handle_hand_motor, joint_limits, soft_joint_limits);
  getJointLimits("xtion_tilt_joint", nh_, joint_limits);
  joint_limits_interface::PositionJointSoftLimitsHandle joint_limit_xtion_tilt(
      pos_handle_xtion_tilt, joint_limits, soft_joint_limits);
  jnt_limit_interface_.registerHandle(joint_limit_arm_roll);
  jnt_limit_interface_.registerHandle(joint_limit_arm_flex);
  jnt_limit_interface_.registerHandle(joint_limit_elbow_flex);
  jnt_limit_interface_.registerHandle(joint_limit_wrist_flex);
  jnt_limit_interface_.registerHandle(joint_limit_hand_motor);
  jnt_limit_interface_.registerHandle(joint_limit_xtion_tilt);
  registerInterface(&jnt_limit_interface_);
}

void SobitEducationControl::writeInitialJoint() {
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("arm_roll_joint");
  traj.joint_names.push_back("arm_flex_joint");
  traj.joint_names.push_back("elbow_flex_joint");
  traj.joint_names.push_back("wrist_flex_joint");
  traj.joint_names.push_back("hand_motor_joint");
  traj.joint_names.push_back("xtion_tilt_joint");
  traj.points.resize(1);
  traj.points[0].positions.resize(6);
  nh_.getParam("/initial_joint/arm_roll_joint", traj.points[0].positions[0]);
  nh_.getParam("/initial_joint/arm_flex_joint", traj.points[0].positions[1]);
  nh_.getParam("/initial_joint/elbow_flex_joint", traj.points[0].positions[2]);
  nh_.getParam("/initial_joint/wrist_flex_joint", traj.points[0].positions[3]);
  nh_.getParam("/initial_joint/hand_motor_joint", traj.points[0].positions[4]);
  nh_.getParam("/initial_joint/xtion_tilt_joint", traj.points[0].positions[5]);
  traj.points[0].time_from_start = ros::Duration(0.0);
  writeDynamixelMotors(traj);
}

void SobitEducationControl::read(ros::Time time, ros::Duration period) {
  sensor_msgs::JointState pose = readDynamixelMotors();
  for (int i = 0; i < pose.name.size(); i++) {
    if (pose.name[i] == "arm_roll_joint") {
      pos_[0] = pose.position[i];
    } else if (pose.name[i] == "arm_flex_joint") {
      pos_[1] = pose.position[i];
    } else if (pose.name[i] == "elbow_flex_joint") {
      pos_[2] = pose.position[i];
    } else if (pose.name[i] == "wrist_flex_joint") {
      pos_[3] = pose.position[i];
    } else if (pose.name[i] == "hand_motor_joint") {
      pos_[4] = pose.position[i];
    } else if (pose.name[i] == "xtion_tilt_joint") {
      pos_[5] = pose.position[i];
    }
  }
}

void SobitEducationControl::write(ros::Time time, ros::Duration period) {
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("arm_roll_joint");
  traj.joint_names.push_back("arm_flex_joint");
  traj.joint_names.push_back("elbow_flex_joint");
  traj.joint_names.push_back("wrist_flex_joint");
  traj.joint_names.push_back("hand_motor_joint");
  traj.joint_names.push_back("xtion_tilt_joint");
  traj.points.resize(1);
  traj.points[0].positions.resize(6);
  traj.points[0].positions[0]    = cmd_[0];
  traj.points[0].positions[1]    = cmd_[1];
  traj.points[0].positions[2]    = cmd_[2];
  traj.points[0].positions[3]    = cmd_[3];
  traj.points[0].positions[4]    = cmd_[4];
  traj.points[0].positions[5]    = cmd_[5];
  traj.points[0].time_from_start = ros::Duration(0.0);
  writeDynamixelMotors(traj);
}