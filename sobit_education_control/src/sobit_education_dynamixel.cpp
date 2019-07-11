#include <sobit_education_control/sobit_education_hw.h>

SobitEducationDynamixel::SobitEducationDynamixel() {
  used_dynamixel_id[1]  = 1;
  used_dynamixel_id[2]  = 2;
  used_dynamixel_id[16] = 16;
  used_dynamixel_id[17] = 17;
  used_dynamixel_id[18] = 18;
  used_dynamixel_id[19] = 19;
  used_dynamixel_id[20] = 20;

  used_dynamixel_name[1]  = "xtion_pan_joint";
  used_dynamixel_name[2]  = "xtion_tilt_joint";
  used_dynamixel_name[16] = "arm_roll_joint";
  used_dynamixel_name[17] = "arm_flex_joint";
  used_dynamixel_name[18] = "elbow_flex_joint";
  used_dynamixel_name[19] = "wrist_flex_joint";
  used_dynamixel_name[20] = "hand_motor_joint";
}

SobitEducationDynamixel::~SobitEducationDynamixel() {}

void SobitEducationDynamixel::initializeDynamixel() {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      setTorqueEnable(i);
      setAcceleration(i, acceleration_val);
      setVelocity(i, velocity_val);
      setGrouopRead(i);
      setPositionIGain(i);
      if (i == 20) setTorqueLimit(i);
    }
  }
}

void SobitEducationDynamixel::writeDynamixelMotors(const trajectory_msgs::JointTrajectory &pose) {
  for (int i = 0; i < pose.joint_names.size(); i++) {
    int joint_id = getJointNumber(pose.joint_names[i]);
    if (!joint_id) {
      continue;
    }
    int joint_pos = toBit(joint_id, pose.points[0].positions[i]);
    if (std::abs(joint_pos == saved_dxl_goal_position[joint_id])) {
      continue;
    }
    addPositionToStorage(joint_id, joint_pos);
  }
  writeGoalPositon();
}

sensor_msgs::JointState SobitEducationDynamixel::readDynamixelMotors() {
  sensor_msgs::JointState pose;
  dxl_comm_result = readPositonGroup.txRxPacket();
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_id[i]) {
      std::string joint_name    = used_dynamixel_name[i];
      int         dynamixel_pos = readCurrentPosition(used_dynamixel_id[i]);
      float       joint_pos     = toRad(joint_name, dynamixel_pos);
      if (dynamixel_pos == -1) {
        // joint_pos = toRad(joint_name, saved_dynamixel_position[i]);
      } else {
        saved_dxl_goal_position[i] = dynamixel_pos;
        pose.name.push_back(joint_name);
        pose.position.push_back(joint_pos);
      }
      pose.name.push_back(joint_name);
      pose.position.push_back(joint_pos);
    }
  }
  return pose;
}

int SobitEducationDynamixel::getJointNumber(std::string joint_name) {
  for (int i = 0; i < 30; i++) {
    if (used_dynamixel_name[i] == joint_name) {
      return used_dynamixel_id[i];
    }
  }
  return 0;
}

float SobitEducationDynamixel::toBit(int id, float rad) {
  if (id == 16) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 17) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 18) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 19) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 20) {
    return 1449 + rad / (M_PI * 2) * 4096;
  } else if (id == 1) {
    return 2048 + rad / (M_PI * 2) * 4096;
  } else if (id == 2) {
    return 2048 + rad / (M_PI * 2) * 4096;
  }
  return -1;
}

float SobitEducationDynamixel::toRad(std::string joint_name, int bit) {
  if (joint_name == "arm_roll_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "arm_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "elbow_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "wrist_flex_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "hand_motor_joint") {
    return (bit - 1449) * ((M_PI * 2) / 4096);
  } else if (joint_name == "xtion_tilt_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  } else if (joint_name == "xtion_pan_joint") {
    return (bit - 2048) * ((M_PI * 2) / 4096);
  }
}