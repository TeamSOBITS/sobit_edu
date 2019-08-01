#include <queue>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <sobit_common_msg/current_ctrl.h>
#include "sobit_education_control/dynamixel_control.h"
#include "sobit_education_control/dynamixel_port_control.h"
#include "sobit_education_control/dynamixel_setting.h"

namespace {
typedef struct {
  uint8_t  dxl_id;
  uint32_t current_val;
} CurrentRequest;

std::queue<CurrentRequest>                    current_request_queue;
dynamixel_port_control::DynamixelPortControl* driver_addr;

void currentCtrlCallback(const sobit_common_msg::current_ctrl msg) {
  std::string target_joint_name  = msg.joint_name;
  uint32_t    target_current_val = msg.current;
  for (std::vector<dynamixel_control::DynamixelControl>::iterator it = driver_addr->joint_list_.begin();
       it != driver_addr->joint_list_.end();
       it++) {
    if (it->getJointName() == target_joint_name) {
      CurrentRequest current_req;
      current_req.dxl_id      = it->getDxlId();
      current_req.current_val = target_current_val;
      current_request_queue.push(current_req);
      break;
    }
  }
}

}  // namespace
int main(int argc, char** argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle                     nh;
  ros::Subscriber                     sub_current_ctrl = nh.subscribe("current_ctrl", 10, &currentCtrlCallback);
  dynamixel_setting::DynamixelSetting setting(nh);
  if (!setting.load()) {
    return -1;
  }
  ROS_INFO("debug func %s line %d.", __func__, __LINE__);
  dynamixel_port_control::DynamixelPortControl sobit_edu(nh, setting);
  controller_manager::ControllerManager        cm(&sobit_edu, nh);
  driver_addr = &sobit_edu;
  sobit_edu.initializeSettingParam();
  sobit_edu.startUpPosition();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time     t  = sobit_edu.getTime();
  ros::Duration dt = sobit_edu.getDuration(t);
  while (ros::ok()) {
    dt = sobit_edu.getDuration(t);
    t  = sobit_edu.getTime();

    sobit_edu.read(t, dt);
    cm.update(t, dt);
    sobit_edu.write(t, dt);

    while (current_request_queue.size() > 0) {
      CurrentRequest req = current_request_queue.front();
      bool           res = sobit_edu.setCurrentLimit(req.dxl_id, req.current_val);
      if (res) {
        current_request_queue.pop();
      } else {
        break;
      }
    }
  }
  spinner.stop();
}