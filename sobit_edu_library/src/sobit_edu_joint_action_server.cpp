#include "sobit_edu_library/sobit_edu_joint_action_server.hpp"

namespace sobit_edu{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_joints_ = rclcpp_action::create_server<MoveJoint>(
      this,
      "move_joint",
      std::bind(&JointActionServer::handle_move_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_joints_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_joints_accepted, this, std::placeholders::_1));
  this->action_server_move_to_pose_ = rclcpp_action::create_server<MoveToPose>(
      this,
      "move_to_pose",
      std::bind(&JointActionServer::handle_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointActionServer::handle_move_to_pose_cancel, this, std::placeholders::_1),
      std::bind(&JointActionServer::handle_move_to_pose_accepted, this, std::placeholders::_1));
  this->service_server_move_hand_to_coord_ = this->create_service<MoveHandToTargetCoord>(
      "move_hand_to_coord",
      std::bind(&JointActionServer::serve_move_hand_to_coord, this, std::placeholders::_1, std::placeholders::_2));
  this->service_server_move_hand_to_tf_ = this->create_service<MoveHandToTargetTF>(
      "move_hand_to_tf",
      std::bind(&JointActionServer::serve_move_hand_to_tf, this, std::placeholders::_1, std::placeholders::_2));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
  this->pub_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory_controller/joint_trajectory", qos_profile);


  //Declare the pose parameters

  this->declare_parameter("poses", std::vector<std::string>());
  auto pose_names = this->get_parameter("poses").as_string_array();

  poses_.clear();
  for (auto pose_name : pose_names) {
    // Declare parameters for each pose
    this->declare_parameter(pose_name + ".arm_shoulder_roll" , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_shoulder_pitch", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_elbow_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_forearm_roll"  , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_pitch"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".arm_wrist_roll"    , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".hand"              , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_camera_pan"   , rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter(pose_name + ".head_camera_tilt"  , rclcpp::PARAMETER_DOUBLE);

    // Read parameters for each pose
    PoseParams params;
    params.pose_name           = pose_name;
    params.arm_shoulder_roll = this->get_parameter(pose_name + ".arm_shoulder_roll").as_double();
    params.arm_shoulder_pitch  = this->get_parameter(pose_name + ".arm_shoulder_pitch").as_double();
    params.arm_elbow_pitch    = this->get_parameter(pose_name + ".arm_elbow_pitch").as_double();
    params.arm_forearm_roll    = this->get_parameter(pose_name + ".arm_forearm_roll").as_double();
    params.arm_wrist_pitch              = this->get_parameter(pose_name + ".arm_wrist_pitch").as_double();
    params.arm_wrist_roll = this->get_parameter(pose_name + ".arm_wrist_roll").as_double();
    params.hand  = this->get_parameter(pose_name + ".hand").as_double();
    params.head_camera_pan    = this->get_parameter(pose_name + ".head_camera_pan").as_double();
    params.head_camera_tilt    = this->get_parameter(pose_name + ".head_camera_tilt").as_double();
    poses_.push_back(params);
  }

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been initialized.");
}
JointActionServer::~JointActionServer()
{
  this->action_server_move_joints_.reset();
  this->action_server_move_to_pose_.reset();

  this->sub_joint_state_.reset();
  this->pub_joint_control_.reset();

  RCLCPP_INFO(this->get_logger(), "JointActionServer has been terminated.");
}


rclcpp_action::GoalResponse JointActionServer::handle_move_joints_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveJoint::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse JointActionServer::handle_move_to_pose_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse JointActionServer::handle_move_joints_cancel(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse JointActionServer::handle_move_to_pose_cancel(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void JointActionServer::handle_move_joints_accepted(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_joints, this, std::placeholders::_1), goal_handle}.detach();
}

void JointActionServer::handle_move_to_pose_accepted(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&JointActionServer::exe_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
}


void JointActionServer::exe_move_joints(
  const std::shared_ptr<GoalHandleMoveJoints> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveJoint::Result>();

  // Check if the number of joint names and joint rad are the same
  if (goal->target_joint_names.size() != goal->target_joint_rad.size()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal request. The number of joint names and joint rad are different");
    result->success = false;
    result->message = "Invalid goal request. The number of joint names and joint rad are different";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Check if the joint names are valid
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    if (std::find(JointNames.begin(), JointNames.end(), goal->target_joint_names[i]) == JointNames.end()) {
      RCLCPP_ERROR(this->get_logger(), "The joint name does not exist: %s", goal->target_joint_names[i].c_str());
      result->success = false;
      result->message = "The joint name does not exist: " + goal->target_joint_names[i];
      result->total_elapsed_time.sec = 0;
      result->total_elapsed_time.nanosec = 0;
      goal_handle->abort(result);
      return;
    }
  }

  // TODO: Check if the joint rad are within the joint limits

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      this->pub_joint_control_->publish(set_joints({}, {}, dt));

      return;
    }

    auto feedback = std::make_shared<MoveJoint::Feedback>();
    feedback->current_joint_names = goal->target_joint_names;
    for (const auto &joint_name : goal->target_joint_names) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();

  }

  // Check if goal was reached
  for (size_t i = 0; i < goal->target_joint_names.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[goal->target_joint_names[i]] - goal->target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::exe_move_to_pose(
  const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPose::Result>();

  // Check if the pose name is valid
  if (std::find_if(poses_.begin(), poses_.end(), [&](const PoseParams &pose) { return pose.pose_name == goal->pose_name; }) == poses_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pose name: %s", goal->pose_name.c_str());
    result->success = false;
    result->message = "Invalid pose name: " + goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
    return;
  }

  // Get the target joint rad from the pose name
  std::vector<double> target_joint_rad;
  for (const auto &pose : poses_) {
    if (pose.pose_name == goal->pose_name) {
      target_joint_rad.push_back(pose.arm_shoulder_roll);
      target_joint_rad.push_back(pose.arm_shoulder_pitch);
      target_joint_rad.push_back(pose.arm_elbow_pitch);
      target_joint_rad.push_back(pose.arm_forearm_roll);
      target_joint_rad.push_back(pose.arm_wrist_pitch);
      target_joint_rad.push_back(pose.arm_wrist_roll);
      target_joint_rad.push_back(pose.hand);
      target_joint_rad.push_back(pose.head_camera_pan);
      target_joint_rad.push_back(pose.head_camera_tilt);
      break;
    }
  }

  if (target_joint_rad.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to not find the pose name : %s", goal->pose_name.c_str());

    result->success = false;
    result->message = "[FAIL] Failed to not find the pose name : " +  goal->pose_name;
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);
  }

  // Publish the joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory = set_joints(JointNames, target_joint_rad, goal->time_allowance);

  try {
    this->pub_joint_control_->publish(joint_trajectory);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish the joint trajectory: %s", ex.what());

    result->success = false;
    result->message = "[FAIL] Failed to publish the joint trajectory";
    result->total_elapsed_time.sec = 0;
    result->total_elapsed_time.nanosec = 0;
    goal_handle->abort(result);

    return;
  }

  // Publish feedback
  auto start_time = this->now();
  // rclcpp::Rate loop_rate(10);

  while (this->now() - start_time < goal->time_allowance) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");

      result->success = false;
      result->message = "[CANCEL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->canceled(result);

      builtin_interfaces::msg::Duration dt;
      dt.sec = 0;
      dt.nanosec = static_cast<uint32_t>(0.1 * 10E9);
      this->pub_joint_control_->publish(set_joints({}, {}, dt));
  
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = JointNames;
    for (const auto &joint_name : JointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < JointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[JointNames[i]] - target_joint_rad[i]) > 0.1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal");

      result->success = false;
      result->message = "[FAIL] Failed to reach the goal";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
      goal_handle->abort(result);

      return;
    }
  }

  // Clear the current joint state
  curt_joint_state_.clear();

  // Publish the result
  result->message = "[SUCCESS] Goal has been succeeded";
  result->success = true;
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}

void JointActionServer::serve_move_hand_to_coord(
  const std::shared_ptr<MoveHandToTargetCoord::Request> request,
  std::shared_ptr<MoveHandToTargetCoord::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->target_coord.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  // Transform to robot base from 'sobit_edu/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      request->target_coord, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  
  target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);


  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (z: 0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::serve_move_hand_to_tf(
  const std::shared_ptr<MoveHandToTargetTF::Request> request,
  std::shared_ptr<MoveHandToTargetTF::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  geometry_msgs::msg::TransformStamped goal_coord_shift;


  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->target_frame, request->tf_differential.header.frame_id,
      tf2::TimePointZero);

    geometry_msgs::msg::Vector3 euler_target, euler_shift;
    euler_target = get_euler_from_quat(goal_coord_shift.transform.rotation);
    euler_shift = get_euler_from_quat(request->tf_differential.transform.rotation);
    euler_target.x += euler_shift.x;
    euler_target.y += euler_shift.y;
    euler_target.z += euler_shift.z;

    goal_coord_shift.transform.translation.x += request->tf_differential.transform.translation.x;
    goal_coord_shift.transform.translation.y += request->tf_differential.transform.translation.y;
    goal_coord_shift.transform.translation.z += request->tf_differential.transform.translation.z;
    goal_coord_shift.transform.rotation = get_quat_from_euler(euler_target);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform: %s to %s: %s", request->target_frame.c_str(), request->tf_differential.header.frame_id.c_str(),ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform: " + request->target_frame + " to: " + request->tf_differential.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // Transform to robot base from 'sobit_edu/base_footprint'
  try{
    goal_coord = tf_buffer_->transform(
      goal_coord_shift, goal_coord.header.frame_id,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform coords to %s: %s", goal_coord.header.frame_id.c_str(), ex.what());

    response->success = false;
    response->message = "[FAIL] Could not transform coords to " + goal_coord.header.frame_id;
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  // target_yawにロボットの回転角度を代入
  // calculate the target_yaw to move base of grasping object 
  double target_linear, target_yaw;
  target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);

  // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる


  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_wrist_pitch_joint"};
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, target_yaw);

  // If inverse kinematics is outside the range of possible
  // もし逆運動学可能範囲外ならば・・・
  if (target_joint_rad.size() == 0) {

    response->success = false;
    // TODO //
    response->message = "[FAIL] The target position is too low or tall (z: 0.3[m] <= Grasp Able <= 0.8[m])";
    response->target_joint_names.clear();
    response->target_joint_rad.clear();

    return;
  }

  geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad, target_yaw);

  if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
    target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  } else {
    target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  }

  response->move_pose.position.x = target_linear;
  response->move_pose.position.y = 0.0;
  response->move_pose.position.z = 0.0;

  geometry_msgs::msg::Vector3 euler;
  euler.x = 0.0;
  euler.y = 0.0;
  euler.z = target_yaw;
  response->move_pose.orientation = get_quat_from_euler(euler);

  response->success = true;
  response->message = "[SUCCESS] The coord is grasp able.";
  response->target_joint_names = target_joint_names;
  response->target_joint_rad = target_joint_rad;

  return;
}

void JointActionServer::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "Received joint state");

  for (size_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "arm_shoulder_pitch_sub_joint") continue;  // Skip sub joints

    this->curt_joint_state_[msg->name[i]] = msg->position[i];
  }
}

trajectory_msgs::msg::JointTrajectory JointActionServer::set_joints(
  const std::vector<std::string> &target_joint_names,
  const std::vector<double> &target_joint_rad,
  const builtin_interfaces::msg::Duration &time_allowance)
{
  // Get current joint state from kCurrentJointState
  std::vector<double> full_target_joint_rad;
  for (size_t i = 0; i < JointNames.size(); i++) {
    full_target_joint_rad.push_back(this->curt_joint_state_[JointNames[i]]);
  }
  
  // Update the target joint rad
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(JointNames.begin(), JointNames.end(), target_joint_names[i]);
    full_target_joint_rad[std::distance(JointNames.begin(), it)] = target_joint_rad[i];
  }

  auto joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  joint_trajectory.header.stamp = this->now();
  joint_trajectory.points.resize(1);
  joint_trajectory.points[0].time_from_start = time_allowance;
  for (size_t i = 0; i < JointNames.size(); i++) {
    joint_trajectory.points[0].positions.push_back(full_target_joint_rad[i]);
    joint_trajectory.joint_names.push_back(JointNames[i]);

    // Add sub joints
    if (joint_trajectory.joint_names[i] == JointNames[JointIds::ArmShoulderPitchJoint]) {
      joint_trajectory.points[0].positions.push_back(-full_target_joint_rad[i]);
      joint_trajectory.joint_names.push_back("arm_shoulder_pitch_sub_joint");
    }
  }

  return joint_trajectory;
}

// ここは，もしも今後逆運動学が3次元に発展したときに，それに対応させるために3次元での順運動学を算出
geometry_msgs::msg::TransformStamped JointActionServer::forward_kinematics(
  const std::vector<double> &target_joint_rad,
  const double target_yaw)
{
}

std::vector<double> JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord,  // 'goal_coord' is the coordinates of robot base.
  const double target_yaw)
{
}

} // namespace sobit_edu
