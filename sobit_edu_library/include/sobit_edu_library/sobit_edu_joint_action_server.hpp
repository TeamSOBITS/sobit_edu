#include <map>

#include "sobits_interfaces/action/move_joint.hpp"
#include "sobits_interfaces/action/move_to_pose.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_coord.hpp"
// #include "sobits_interfaces/action/move_hand_to_target_tf.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_coord.hpp"
#include "sobits_interfaces/srv/get_hand_to_target_tf.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.h"
#include "geometry_msgs/msg/vector3.h"
#include "geometry_msgs/msg/point.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Dense>

#include "sobit_edu_library/ik_library.hpp"


namespace sobit_edu
{

// 運動学の方式(パラメータ kinematics_method で選ぶ)
enum class KinematicsMethod
{
  Trigonometric,  // "trigonometric" (既定): 2次元・三角関数(IKLibrary::inverse_kinematics_trigonometric)
  Jacobian,       // "jacobian"            : 2次元・ヤコビ行列(IKLibrary::inverse_kinematics_jacobian)
  IKLibrary,      // "ik_library"          : 8自由度・LM法(IKLibrary::inverse_kinematics)
};

struct PoseParams
{
  std::string pose_name;
  double arm_shoulder_roll;
  double arm_shoulder_pitch;
  double arm_elbow_pitch;
  double arm_forearm_roll;
  double arm_wrist_pitch;
  double arm_wrist_roll;
  double hand;
  double head_camera_pan;
  double head_camera_tilt;
};

class JointActionServer : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using MoveToPose = sobits_interfaces::action::MoveToPose;
  using GetHandToTargetCoord = sobits_interfaces::srv::GetHandToTargetCoord;
  using GetHandToTargetTF = sobits_interfaces::srv::GetHandToTargetTF;

  using GoalHandleMoveJoints = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveJoint>;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<sobits_interfaces::action::MoveToPose>;


  explicit JointActionServer(const rclcpp::NodeOptions & options);
  ~JointActionServer();

  geometry_msgs::msg::Vector3 get_euler_from_quat(
    const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion get_quat_from_euler(
    const geometry_msgs::msg::Vector3& rpy);
  // jazzy-devel 版の2次元運動学(q = {shoulder_pitch, elbow_pitch, wrist_pitch})
  geometry_msgs::msg::TransformStamped forward_kinematics(
    const std::vector<double> &target_joint_rad,
    const double target_yaw);
  std::vector<double> inverse_kinematics_trigonometric(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    const double target_yaw);
  std::vector<double> inverse_kinematics_jacobian(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    const double target_yaw);
  trajectory_msgs::msg::JointTrajectory set_joints(
    const std::vector<std::string> &target_joint_names,
    const std::vector<double> &target_joint_rad,
    const builtin_interfaces::msg::Duration &time_allowance,
    const std::string &pose_name);

private:
  const std::vector<std::string> kArmJointNames = {
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
  };
  const std::vector<std::string> kHandJointNames = {
    "hand_joint"
  };
  const std::vector<std::string> kHeadJointNames = {
    "head_camera_pan_joint",
    "head_camera_tilt_joint"
  };

  const std::vector<std::string> kJointNames = {
    "arm_shoulder_roll_joint",
    "arm_shoulder_pitch_joint",
    "arm_elbow_pitch_joint",
    "arm_forearm_roll_joint",
    "arm_wrist_pitch_joint",
    "arm_wrist_roll_joint",
    "hand_joint",
    "head_camera_pan_joint",
    "head_camera_tilt_joint",
  };

  // jazzy-devel 版の2次元運動学で使う寸法(IKLibrary の8自由度モデルとは別に持つ)
  static constexpr double base_to_shoulder_x = 0.12561;
  static constexpr double base_to_shoulder_z = 0.55505;
  static constexpr double arm_upper_link = 0.128;
  static constexpr double arm_lower_link = 0.146;
  static constexpr double arm_gripper_link = 0.1834;

  std::vector<PoseParams> poses_;
  std::map<std::string, double> init_joint_state_;
  std::map<std::string, double> curt_joint_state_;

  rclcpp_action::Server<MoveJoint>::SharedPtr action_server_move_joints_;
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_move_to_pose_;
  rclcpp::Service<GetHandToTargetCoord>::SharedPtr service_server_get_hand_to_coord_;
  rclcpp::Service<GetHandToTargetTF>::SharedPtr service_server_get_hand_to_tf_;

  rclcpp_action::GoalResponse handle_move_joints_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_move_to_pose_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_move_joints_cancel(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  rclcpp_action::CancelResponse handle_move_to_pose_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void handle_move_joints_accepted(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void handle_move_to_pose_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

  void exe_move_joints(const std::shared_ptr<GoalHandleMoveJoints> goal_handle);
  void exe_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void serve_get_hand_to_coord(const std::shared_ptr<GetHandToTargetCoord::Request> request, std::shared_ptr<GetHandToTargetCoord::Response> response);
  void serve_get_hand_to_tf(const std::shared_ptr<GetHandToTargetTF::Request> request, std::shared_ptr<GetHandToTargetTF::Response> response);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_hand_joint_control_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_joint_control_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  IKLibrary ik_library_;
  KinematicsMethod kinematics_method_ = KinematicsMethod::Trigonometric;

  // get_hand_to_coord / get_hand_to_tf 共通: base_footprint 基準の目標から関節角と足回りの移動量を求め，
  // 応答に詰める。kinematics_method_ に応じて下の方式別の関数を呼び分ける
  template<typename ResponseT>
  void solve_hand_to_target(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    ResponseT &response);

  // 方式別の逆運動学。成功なら true を返し，関節名・関節角・足回りの移動量(move_pose)を埋める
  bool solve_hand_to_target_2dof(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    std::vector<std::string> &target_joint_names,
    std::vector<double> &target_joint_rad,
    geometry_msgs::msg::Pose &move_pose);
  bool solve_hand_to_target_8dof(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    std::vector<std::string> &target_joint_names,
    std::vector<double> &target_joint_rad,
    geometry_msgs::msg::Pose &move_pose);

  // 足回り(ベース)の移動量。2自由度方式: 旋回=目標の方位，前後=腕のFKで届いた手先と目標の差
  geometry_msgs::msg::Pose calc_base_motion_2dof(
    const geometry_msgs::msg::TransformStamped &goal_coord,
    const std::vector<double> &target_joint_rad,
    double target_yaw);
  // 足回り(ベース)の移動量。8自由度方式: IKLibrary の解の q[6](旋回), q[7](前後)をそのまま使う
  geometry_msgs::msg::Pose calc_base_motion_8dof(
    const std::vector<double> &ik_result);

}; // class JointActionServer

inline geometry_msgs::msg::Vector3 JointActionServer::get_euler_from_quat(
  const geometry_msgs::msg::Quaternion& msg_quat)
{
  tf2::Quaternion tf_quat;
  geometry_msgs::msg::Vector3 euler;

  tf2::fromMsg(msg_quat, tf_quat);
  tf_quat.normalize();
  tf2::Matrix3x3(tf_quat).getRPY(euler.x, euler.y, euler.z);

  return euler;  
}

inline geometry_msgs::msg::Quaternion JointActionServer::get_quat_from_euler(
  const geometry_msgs::msg::Vector3& euler)
{
  tf2::Quaternion tf_quat;

  tf_quat.setRPY(euler.x, euler.y, euler.z);

  return tf2::toMsg(tf_quat);
}

} // namespace sobit_edu

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_edu::JointActionServer)


