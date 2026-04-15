#include "sobit_edu_library/sobit_edu_joint_action_server.hpp"


namespace sobit_edu{

JointActionServer::JointActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("joint_action_server", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
  // b((VectorXd(6)<<1.0/arm_upper_link, 1.0/arm_upper_link, 1.0/arm_upper_link, 1.0/(2*M_PI), 1.0/(2*M_PI), 1.0/(2*M_PI)).finished()),///(2*M_PI)
  //誤差重み行列
  W_E (b.asDiagonal()),//bを対角行列として誤差重み行列とする
  //減衰因子行列
  W_N_bar (b_bar.asDiagonal())//(MatrixXd::Identity(8, 8)*0.001*arm_upper_link*arm_upper_link)
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
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
  this->service_server_get_hand_to_coord_ = this->create_service<GetHandToTargetCoord>(
      "get_hand_to_coord",
      std::bind(&JointActionServer::serve_get_hand_to_coord, this, std::placeholders::_1, std::placeholders::_2));
  this->service_server_get_hand_to_tf_ = this->create_service<GetHandToTargetTF>(
      "get_hand_to_tf",
      std::bind(&JointActionServer::serve_get_hand_to_tf, this, std::placeholders::_1, std::placeholders::_2));

  this->sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", qos_profile, std::bind(&JointActionServer::joint_state_callback, this, std::placeholders::_1));
  this->pub_arm_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_position_controller/joint_trajectory", qos_profile);
  this->pub_hand_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "hand_position_controller/joint_trajectory", qos_profile);
  this->pub_head_joint_control_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "head_position_controller/joint_trajectory", qos_profile);


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
  this->pub_arm_joint_control_.reset();
  this->pub_hand_joint_control_.reset();
  this->pub_head_joint_control_.reset();

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
    if (std::find(kJointNames.begin(), kJointNames.end(), goal->target_joint_names[i]) == kJointNames.end()) {
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
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory hand_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory head_joint_trajectory;
  arm_joint_trajectory  = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "arm");
  hand_joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "hand");
  head_joint_trajectory = set_joints(goal->target_joint_names, goal->target_joint_rad, goal->time_allowance, "head");
  
  try {
    if (!arm_joint_trajectory.joint_names.empty())
      this->pub_arm_joint_control_->publish(arm_joint_trajectory);
    if (!hand_joint_trajectory.joint_names.empty())
      this->pub_hand_joint_control_->publish(hand_joint_trajectory);
    if (!head_joint_trajectory.joint_names.empty())
      this->pub_head_joint_control_->publish(head_joint_trajectory);
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
      this->pub_head_joint_control_->publish(set_joints({}, {}, dt, "head"));
      this->pub_arm_joint_control_->publish(set_joints({}, {}, dt, "arm"));
      this->pub_hand_joint_control_->publish(set_joints({}, {}, dt, "hand"));

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
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory hand_joint_trajectory;
  trajectory_msgs::msg::JointTrajectory head_joint_trajectory;
  arm_joint_trajectory  = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "arm");
  hand_joint_trajectory = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "hand");
  head_joint_trajectory = set_joints(kJointNames, target_joint_rad, goal->time_allowance, "head");

  try {
    if (!arm_joint_trajectory.joint_names.empty())
      this->pub_arm_joint_control_->publish(arm_joint_trajectory);
    if (!hand_joint_trajectory.joint_names.empty())
      this->pub_hand_joint_control_->publish(hand_joint_trajectory);
    if (!head_joint_trajectory.joint_names.empty())
      this->pub_head_joint_control_->publish(head_joint_trajectory);
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
  
      return;
    }

    auto feedback = std::make_shared<MoveToPose::Feedback>();
    feedback->current_joint_names = kJointNames;
    for (const auto &joint_name : kJointNames) {
      feedback->current_joint_rad.push_back(this->curt_joint_state_[joint_name]);
    }
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

    goal_handle->publish_feedback(feedback);

    // rclcpp::spin_some(this->get_node_base_interface());
    // loop_rate.sleep();
  }

  // Check if goal was reached
  for (size_t i = 0; i < kJointNames.size(); i++) {
    // TODO: set tolerance with parameter or msg
    if (std::abs(this->curt_joint_state_[kJointNames[i]] - target_joint_rad[i]) > 0.1) {
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

void JointActionServer::serve_get_hand_to_coord(
  const std::shared_ptr<GetHandToTargetCoord::Request> request,
  std::shared_ptr<GetHandToTargetCoord::Response> response)
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
  // double target_linear, target_yaw;
  
  // target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);


  // // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる

  // geometry_msgs::msg::TransformStamped hand_coord;
  // hand_coord.header = goal_coord.header;
  // hand_coord.transform.translation.x = goal_coord.transform.translation.x*std::cos(-target_yaw) - goal_coord.transform.translation.y*std::sin(-target_yaw);
  // hand_coord.transform.translation.y = goal_coord.transform.translation.y*std::cos(-target_yaw) + goal_coord.transform.translation.x*std::sin(-target_yaw);
  // hand_coord.transform.translation.z = goal_coord.transform.translation.z;

  // geometry_msgs::msg::Vector3 euler_target_yaw;
  // euler_target_yaw = get_euler_from_quat(hand_coord.transform.rotation);
  // euler_target_yaw.z -= target_yaw;
  // hand_coord.transform.rotation = get_quat_from_euler(euler_target_yaw);
  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_roll_joint", "arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_forearm_roll_joint", "arm_wrist_pitch_joint", "arm_wrist_roll_joint"};

  //  Get current joint state from kCurrentJointState
  std::vector<double> current_joint_rad;
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(kJointNames.begin(), kJointNames.end(), target_joint_names[i]);
    if (it == kJointNames.end()) return;
    // current_joint_rad.push_back(this->curt_joint_state_[kJointNames[i]]);
    current_joint_rad.push_back(0.0);
  }

  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, current_joint_rad);

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
  double target_yaw = target_joint_rad[6];
  double target_linear = target_joint_rad[7];

  std::vector<double>  target_arm_rad(target_joint_rad.begin(), target_joint_rad.begin() + std::min<size_t>(6, target_joint_rad.size()));
  Map<VectorXd> target_joint_rad_eigen(target_joint_rad.data(), target_joint_rad.size());
  // geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_joint_rad_eigen);

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
  response->target_joint_rad = target_arm_rad;

  return;
}

void JointActionServer::serve_get_hand_to_tf(
  const std::shared_ptr<GetHandToTargetTF::Request> request,
  std::shared_ptr<GetHandToTargetTF::Response> response)
{

  // Get namespace
  geometry_msgs::msg::TransformStamped goal_coord;
  goal_coord.header = request->tf_differential.header;
  goal_coord.header.frame_id = std::string(this->get_namespace()).substr(1) + "/base_footprint";

  geometry_msgs::msg::TransformStamped goal_coord_shift;


  // Transform the target frame based on the differential tf
  try {
    goal_coord_shift = tf_buffer_->lookupTransform(
      request->tf_differential.header.frame_id, request->target_frame, 
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
  // double target_linear, target_yaw;
  // target_yaw = std::atan2(goal_coord.transform.translation.y,goal_coord.transform.translation.x);

  // // 3次元の逆運動学が完成したらtarget_yawはある一定の条件で0(=回転する必要なし)になる
  // if (std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2)) > std::sqrt(std::pow(0.5, 2)+std::pow(0.0, 2))) {
  //   target_linear =  std::sqrt(std::pow(goal_coord.transform.translation.x - 0.5,2) + std::pow(goal_coord.transform.translation.y,2));
  // } else {
  //   target_linear = -std::sqrt(std::pow(goal_coord.transform.translation.x - 0.5,2) + std::pow(goal_coord.transform.translation.y,2));
  // }

  // Inverse kinematics to get the target joint rad
  std::vector<std::string> target_joint_names = {"arm_shoulder_roll_joint", "arm_shoulder_pitch_joint","arm_elbow_pitch_joint", "arm_forearm_roll_joint", "arm_wrist_pitch_joint", "arm_wrist_roll_joint"};
  std::vector<double> current_joint_rad;
  for (size_t i = 0; i < target_joint_names.size(); i++) {
    auto it = std::find(kJointNames.begin(), kJointNames.end(), target_joint_names[i]);
    if (it == kJointNames.end()) return;
    // current_joint_rad.push_back(this->curt_joint_state_[kJointNames[i]]);
    current_joint_rad.push_back(0.0);

  }
  std::vector<double> target_joint_rad = inverse_kinematics(goal_coord, current_joint_rad);

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

  double target_yaw = target_joint_rad[6];
  double target_linear = target_joint_rad[7];

  std::vector<double>  target_arm_rad(target_joint_rad.begin(), target_joint_rad.begin() + std::min<size_t>(6, target_joint_rad.size()));
  Map<VectorXd> target_joint_rad_eigen(target_joint_rad_eigen.data(), target_joint_rad_eigen.size());
  // geometry_msgs::msg::TransformStamped hand_pose = forward_kinematics(target_arm_rad_eigen);

  // if (std::sqrt(std::pow(hand_pose.transform.translation.x,2)+std::pow(hand_pose.transform.translation.y,2)) < std::sqrt(std::pow(goal_coord.transform.translation.x,2)+std::pow(goal_coord.transform.translation.y,2))) {
  //   target_linear =  std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  // } else {
  //   target_linear = -std::sqrt(std::pow(hand_pose.transform.translation.x - goal_coord.transform.translation.x,2) + std::pow(hand_pose.transform.translation.y - goal_coord.transform.translation.y,2));
  // }

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
  response->target_joint_rad = target_arm_rad;

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
  const builtin_interfaces::msg::Duration &time_allowance,
  const std::string &group_name)
{
  auto joint_trajectory  = trajectory_msgs::msg::JointTrajectory();
  trajectory_msgs::msg::JointTrajectoryPoint point;

  for (size_t i = 0; i < target_joint_names.size(); i++) {
    // Check if the joint belongs to the specified group
    if (group_name == "arm" &&
        std::find(kArmJointNames.begin(), kArmJointNames.end(), target_joint_names[i]) == kArmJointNames.end()) {
      continue;
    }
    else if (group_name == "hand" &&
        std::find(kHandJointNames.begin(), kHandJointNames.end(), target_joint_names[i]) == kHandJointNames.end()) {
      continue;
    }
    else if (group_name == "head" &&
        std::find(kHeadJointNames.begin(), kHeadJointNames.end(), target_joint_names[i]) == kHeadJointNames.end()) {
      continue;
    }
    joint_trajectory.joint_names.push_back(target_joint_names[i]);
    point.positions.push_back(target_joint_rad[i]);

    // Subjoint to turn opposite direction
    if (target_joint_names[i] == "arm_shoulder_pitch_joint") {
      joint_trajectory.joint_names.push_back("arm_shoulder_pitch_sub_joint");
      point.positions.push_back(-target_joint_rad[i]);
      continue;
    } 
  }

  joint_trajectory.points.push_back(point);
  joint_trajectory.points[0].time_from_start = time_allowance;

  return joint_trajectory;
}

// ここは，もしも今後逆運動学が3次元に発展したときに，それに対応させるために3次元での順運動学を算出
geometry_msgs::msg::TransformStamped JointActionServer::forward_kinematics(const VectorXd& target_joint_rad_eigen){

  geometry_msgs::msg::TransformStamped final_coord;

  //SOBIT_EDU
  DH_modified Trans_base_Y(0.0, 0.0, 0.0, target_joint_rad_eigen(6));

  DH_modified Trans_base_x((target_joint_rad_eigen(7) + base_to_shoulder_x), 0.0, base_to_shoulder_z, 0.0);

  DH_modified Trans_1(0.0, 0.0, 0.0, target_joint_rad_eigen(JointIds::ArmShoulderRollJoint));
  // std::cout <<"0T1"<< std::endl<< Trans_1.Trans << std::endl;
  DH_modified Trans_2(0.0, -M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmShoulderPitchJoint));
  // std::cout <<"1T2"<< std::endl<< Trans_2.Trans << std::endl;
  DH_modified Trans_2_1(arm_shoulder_derr, 0.0, 0.0, -M_PI_2);
  // std::cout << "2T2_1"<< std::endl << Trans_2_1.Trans << std::endl;
  DH_modified Trans_3(arm_upper_link, 0.0, 0.0, target_joint_rad_eigen(JointIds::ArmElbowPitchJoint));
  // std::cout << "2_1T3"<< std::endl << Trans_3.Trans << std::endl;
  DH_modified Trans_4(0.0, -M_PI_2, arm_elbow_to_lower, target_joint_rad_eigen(JointIds::ArmForearmRollJoint));
  // std::cout << "3T4"<< std::endl << Trans_4.Trans << std::endl;
  DH_modified Trans_5(0.0, M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmWristPitchJoint));
  // std::cout << "4T5"<< std::endl << Trans_5.Trans << std::endl;
  DH_modified Trans_6(0.0, -M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmWristRollJoint));
  // std::cout << "5T6"<< std::endl << Trans_6.Trans << std::endl;
  DH_modified Trans_7(0.0, 0.0, arm_gripper_link, 0.0);
  // std::cout << "6TE"<< std::endl << Trans_7.Trans << std::endl;
  DH_modified Trans_8(0.0, -M_PI_2, 0.0, -M_PI_2);
  // std::cout << "E8"<< std::endl << Trans_8.Trans << std::endl;
  DH_modified Trans_9(0.0, -M_PI_2, 0.0, 0.0);
  // std::cout << "88"<< std::endl << Trans_8.Trans << std::endl;

  Matrix4d result = Trans_base_Y.trans_coord*Trans_base_x.trans_coord;
  result = result*Trans_1.trans_coord;
  result = result*Trans_2.trans_coord;
  result = result*Trans_2_1.trans_coord;
  result = result*Trans_3.trans_coord;
  result = result*Trans_4.trans_coord;
  result = result*Trans_5.trans_coord;
  result = result*Trans_6.trans_coord;
  result = result*Trans_7.trans_coord;
  result = result*Trans_8.trans_coord;
  result = result*Trans_9.trans_coord;

  final_coord.transform.translation.x = result(0,3);
  final_coord.transform.translation.y = result(1,3);
  final_coord.transform.translation.z = result(2,3);

  Quaterniond qua_result(result.block<3,3>(0,0));
  // std::cout << qua_result << std::endl;

  final_coord.transform.rotation.w = qua_result.w();
  final_coord.transform.rotation.x = qua_result.x();
  final_coord.transform.rotation.y = qua_result.y();
  final_coord.transform.rotation.z = qua_result.z();
  
  return final_coord;  
}

std::vector<double> JointActionServer::inverse_kinematics(
  const geometry_msgs::msg::TransformStamped &goal_coord,  // 'goal_coord' is the coordinates of robot base.
  const std::vector<double> &current_joint_rad)
{
  MatrixXd W_N(8, 8), P_arm(8, 8), H_arm(8, 8), dr(3,3), R(3,3);
  MatrixXd I_p = MatrixXd::Identity(8,8);
  VectorXd e(6), k_p(8), k_s(8), q(8), delta_q(8);

  q << current_joint_rad[0], current_joint_rad[1], current_joint_rad[2], current_joint_rad[3], current_joint_rad[4], current_joint_rad[5], 0.0, 0.0;

  std::cout << "q" << std::endl << q.transpose() << std::endl;

  do{
  cal_e(e, q, goal_coord);//残差計算
  Matrix<double, 6, 8> J_v = cal_Jv(q);
  cal_P(P_arm, I_p, J_v);
  cal_k(k_p, J_v, e);
  cal_ks(k_s, q);
  cal_W_N(W_N, e);
  cal_H(H_arm, J_v, W_N);
  // std::cout << "J" << std::endl << J_v*P_arm << std::endl;

  delta_q = H_arm.inverse() * k_p + (P_arm * k_s) ;
  // std::cout << "delta_q" << std::endl << delta_q.transpose() << std::endl;

  q += delta_q;
  if (std::abs(q[0]) >= 2*M_PI) q[0] -= 2*M_PI * (int)(q[0] / 2*M_PI);
  if (std::abs(q[1]) >= 2*M_PI) q[1] -= 2*M_PI * (int)(q[1] / 2*M_PI);
  if (std::abs(q[0]) >= M_PI) q[0] -= 2*M_PI * (q[0] / std::abs(q[0]));
  if (std::abs(q[1]) >= M_PI) q[1] -= 2*M_PI * (q[1] / std::abs(q[1]));
  if (std::abs(q[2]) >= 2*M_PI) q[2] -= 2*M_PI * (int)(q[2] / 2*M_PI);
  if (std::abs(q[3]) >= 2*M_PI) q[3] -= 2*M_PI * (int)(q[3] / 2*M_PI);
  if (std::abs(q[2]) >= M_PI) q[2] -= 2*M_PI * (q[2] / std::abs(q[2]));
  if (std::abs(q[3]) >= M_PI) q[3] -= 2*M_PI * (q[3] / std::abs(q[3]));
  if (std::abs(q[4]) >= 2*M_PI) q[4] -= 2*M_PI * (int)(q[4] / 2*M_PI);
  if (std::abs(q[5]) >= 2*M_PI) q[5] -= 2*M_PI * (int)(q[5] / 2*M_PI);
  if (std::abs(q[4]) >= M_PI) q[4] -= 2*M_PI * (q[4] / std::abs(q[4]));
  if (std::abs(q[5]) >= M_PI) q[5] -= 2*M_PI * (q[5] / std::abs(q[5]));
  if (std::abs(q[6]) >= 2*M_PI) q[6] -= 2*M_PI * (int)(q[6] / 2*M_PI);
  if (std::abs(q[6]) >= M_PI) q[6] -= 2*M_PI * (q[6] / std::abs(q[6]));

  if (q[0] >= M_PI/4) q[0] = M_PI/4;
  if (q[0] <= -M_PI/4) q[0] = -M_PI/4;
  if (q[1] >= 3*M_PI/4) q[1] = 3*M_PI/4;
  if (q[1] <= -M_PI_2) q[1] = -M_PI_2;
  if (q[2] >= M_PI_2) q[2] = M_PI_2;
  if (q[2] <= -M_PI_2) q[2] = -M_PI_2;
  if (q[3] >= M_PI) q[3] = M_PI;
  if (q[3] <= -M_PI) q[3] = -M_PI;
  if (q[4] >= M_PI_2) q[4] = M_PI_2;
  if (q[4] <= -M_PI_2) q[4] = -M_PI_2;
  if (q[5] >= M_PI) q[5] = M_PI;
  if (q[5] <= -M_PI) q[5] = -M_PI;
  if (q[6] >= M_PI_2) q[6] = M_PI_2;
  if (q[6] <= -M_PI_2) q[6] = -M_PI_2;

  if (q[7] <= -0.5) q[7] = -0.5;

  iteration++;
  // std::cout << "iteration:" << iteration <<std::endl;
  std::cout << "eの値" << std::endl << e.transpose() << std::endl;
  std::cout << "J(I-P_arm)の値" << std::endl << (J_v* P_arm) << std::endl;

  }while((fabs(e(0)) > threshold_e || fabs(e(1)) > threshold_e || fabs(e(2)) > threshold_e  || fabs(e(3)) > threshold_e  || fabs(e(4)) > threshold_e  || fabs(e(5)) > threshold_e) && (fabs(delta_q(0)) > threshold_q || fabs(delta_q(1)) > threshold_q || fabs(delta_q(2)) > threshold_q || fabs(delta_q(3)) > threshold_q || fabs(delta_q(4)) > threshold_q || fabs(delta_q(5)) > threshold_q || fabs(delta_q(6)) > threshold_q || fabs(delta_q(7)) > threshold_q) && iteration < maxIterations);
  // }while((fabs(e(0)) > threshold_e || fabs(e(1)) > threshold_e || fabs(e(2)) > threshold_e  || fabs(e(3)) > threshold_e  || fabs(e(4)) > threshold_e  || fabs(e(5)) > threshold_e) && iteration < maxIterations);
  // }while(iteration < maxIterations);
  iteration = 0;
  std::vector<double> result_q(q.data(), q.data() + q.size());
  if (iteration >= maxIterations) result_q.clear();

  // std::cout << "result_q" << std::endl << result_q[0] << std::endl
  //                          << std::endl << result_q[1] << std::endl
  //                           << std::endl << result_q[2] << std::endl
  //                            << std::endl << result_q[3] << std::endl
  //                             << std::endl << result_q[4] << std::endl
  //                              << std::endl << result_q[5] << std::endl;
  // q << 0.0,0.0,0.0,0.0,0.0,0.0;

  return result_q;
}

//位置誤差
void JointActionServer::cal_e(VectorXd& e, VectorXd& q_bar, const geometry_msgs::msg::TransformStamped &goal_coord){
  // std::cout << "暫定解q"<< std::endl << q_bar.transpose() << std::endl << "--------------" <<std::endl;

  geometry_msgs::msg::TransformStamped fk = forward_kinematics(q_bar);
  //目標位置と現在の位置の差分を計算
  e(0) = goal_coord.transform.translation.x - fk.transform.translation.x;//(arm_shoulder_derr*std::cos(q_bar(0))*std::cos(q_bar(1))+arm_upper_link*std::cos(q_bar(0))*std::sin(q_bar(1))+arm_lower_link*std::cos(q_bar(0))*std::cos(q_bar(1)+q_bar(2)));
  e(1) = goal_coord.transform.translation.y - fk.transform.translation.y;//(arm_shoulder_derr*std::sin(q_bar(0))*std::cos(q_bar(1))+arm_upper_link*std::sin(q_bar(0))*std::sin(q_bar(1))+arm_lower_link*std::sin(q_bar(0))*std::cos(q_bar(1)+q_bar(2)));
  e(2) = goal_coord.transform.translation.z - fk.transform.translation.z;//(-arm_shoulder_derr*std::sin(q_bar(0))+arm_upper_link*std::cos(q_bar(1))-arm_lower_link*std::sin(q_bar(1)+q_bar(2)));
// 姿勢誤差
  //目標位置と現在野市と差分を計算
  MatrixXd dR(3, 3), R(3, 3), rrq(3, 3);
  VectorXd l(3), m(3), a(3);

  Quaterniond q_dR(
  fk.transform.rotation.w,
  fk.transform.rotation.x,
  fk.transform.rotation.y,
  fk.transform.rotation.z
  );

  Quaterniond q_R(
  goal_coord.transform.rotation.w,
  goal_coord.transform.rotation.x,
  goal_coord.transform.rotation.y,
  goal_coord.transform.rotation.z
  );

  dR = q_dR.toRotationMatrix();

  R = q_R.toRotationMatrix();

  rrq = R * dR.transpose();//姿勢行列

  // std::cout << "rrq" << std::endl << rrq << std::endl;
  l(0) = rrq(2,1)-rrq(1,2);
  l(1) = rrq(0,2)-rrq(2,0);
  l(2) = rrq(1,0)-rrq(0,1);

  m(0) = rrq(0,0) + 1;
  m(1) = rrq(1,1) + 1;
  m(2) = rrq(2,2) + 1;

  bool tani = true;
  bool taikaku = true;

  // Rが対角行列
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      if(i != j && fabs(rrq(i, j)) > 1e-12){ 
        // std::cout << "単位行列でも対角行列でもない" <<  rrq(i, j) << std::endl;
        tani = false;
        taikaku = false;
      }
      if(i == j && rrq(i, j) != 1){
        // std::cout << "対角行列" << std::endl; 
        tani = false;
      }
    }
  }

  if(tani){
        // std::cout << "単位行列" << a << std::endl;
            a.setZero();

  }else if(taikaku){
        a = M_PI_2 * m;
        // std::cout << "対角行列" << std::endl << a.transpose() << std::endl;
  }else{
    // std::cout << "lの値" << std::endl << l << std::endl;
    // std::cout << "l.squaredNorm()" << l.squaredNorm() << std::endl;
        a = (atan2(l.squaredNorm(), rrq(0,0)+rrq(1,1)+rrq(2,2)-1) / l.squaredNorm())* l;
        // a = a ;
        // std::cout << "単位行列でも対角行列でもない" <<std::endl << a << std::endl;
  }

  // Rが対角行列でない
  //回転行列の一致性を評価
  e(3) = a(0);
  e(4) = a(1);
  e(5) = a(2);
// std::cout << "e:" << std::endl << e << std::endl;
  return;
}

//基礎ヤコビ行列を計算する関数
Matrix<double, 6,8> JointActionServer::cal_Jv(const VectorXd &current_joint_rad) {
  Matrix<double, 6,8> J;

  Vector3d e_z;
  e_z << 0.0, 0.0, 1.0;
  
    //SOBIT_EDU 手先
          DH_modified Trans_coord_base_Y(0.0, 0.0, 0.0, current_joint_rad(6));

          DH_modified Trans_coord_base_x((current_joint_rad(7) + base_to_shoulder_x), 0.0, base_to_shoulder_z, 0.0);

          DH_modified Trans_coord_1(0.0, 0.0, 0.0, current_joint_rad(JointIds::ArmShoulderRollJoint));
          // std::cout <<"0T1"<< std::endl<< Trans_coord_1.trans_coord << std::endl;
          DH_modified Trans_coord_2(0.0, -M_PI_2, 0.0, current_joint_rad(JointIds::ArmShoulderPitchJoint));
          // std::cout <<"1T2"<< std::endl<< Trans_coord_2.trans_coord << std::endl;
          DH_modified Trans_coord_2_1(arm_shoulder_derr, 0.0, 0.0, -M_PI_2);
          // std::cout << "2T2_1"<< std::endl << Trans_coord_2_1.trans_coord << std::endl;
          DH_modified Trans_coord_3(arm_upper_link, 0.0, 0.0, current_joint_rad(JointIds::ArmElbowPitchJoint));
          // std::cout << "2_1T3"<< std::endl << Trans_coord_3.trans_coord << std::endl;
          DH_modified Trans_coord_4(0.0, -M_PI_2, arm_elbow_to_lower, current_joint_rad(JointIds::ArmForearmRollJoint));
          // std::cout << "3T4"<< std::endl << Trans_coord_4.trans_coord << std::endl;
          DH_modified Trans_coord_5(0.0, M_PI_2, 0.0, current_joint_rad(JointIds::ArmWristPitchJoint));
          // std::cout << "4T5"<< std::endl << Trans_coord_5.trans_coord << std::endl;
          DH_modified Trans_coord_6(0.0, -M_PI_2, 0.0, current_joint_rad(JointIds::ArmWristRollJoint));
          // std::cout << "5T6"<< std::endl << Trans_coord_6.trans_coord << std::endl;
          DH_modified Trans_coord_7(0.0, 0.0, arm_gripper_link, 0.0);
          // std::cout << "6TE"<< std::endl << Trans_coord_7.trans_coord << std::endl;
          DH_modified Trans_coord_8(0.0, -M_PI_2, 0.0, -M_PI_2);
          // std::cout << "E8"<< std::endl << Trans_8.Trans << std::endl;
          DH_modified Trans_coord_9(0.0, -M_PI_2, 0.0, 0.0);
          // std::cout << "88"<< std::endl << Trans_8.Trans << std::endl;

          Matrix4d result_base_Y = Trans_coord_base_Y.trans_coord;
          Matrix4d result_base_x = result_base_Y*Trans_coord_base_x.trans_coord;
          Matrix4d result_1 = result_base_x*Trans_coord_1.trans_coord;
          Matrix4d result_2 = result_1*Trans_coord_2.trans_coord;
          Matrix4d result_2_1 = result_2*Trans_coord_2_1.trans_coord;
          Matrix4d result_3 = result_2_1*Trans_coord_3.trans_coord;
          Matrix4d result_4 = result_3*Trans_coord_4.trans_coord;
          Matrix4d result_5 = result_4*Trans_coord_5.trans_coord;
          Matrix4d result_6 = result_5*Trans_coord_6.trans_coord;
          Matrix4d result_7 = result_6*Trans_coord_7.trans_coord;
          Matrix4d result_8 = result_7*Trans_coord_8.trans_coord;
          Matrix4d result_9 = result_8*Trans_coord_9.trans_coord;          

          Vector3d p_6_E = result_9.block<3,1>(0,3) - result_6.block<3,1>(0,3);
          Vector3d p_5_E = result_9.block<3,1>(0,3) - result_5.block<3,1>(0,3);
          Vector3d p_4_E = result_9.block<3,1>(0,3) - result_4.block<3,1>(0,3);
          Vector3d p_3_E = result_9.block<3,1>(0,3) - result_3.block<3,1>(0,3);
          Vector3d p_2_E = result_9.block<3,1>(0,3) - result_2.block<3,1>(0,3);
          Vector3d p_1_E = result_9.block<3,1>(0,3) - result_1.block<3,1>(0,3);
          Vector3d p_base_Y_E = result_9.block<3,1>(0,3) - result_base_Y.block<3,1>(0,3);


          
          Vector3d z_base_Y = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_base_x(std::cos(current_joint_rad(6)), std::sin(current_joint_rad(6)), 0.0);

          Vector3d z_1 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_2 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_3 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_4 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_5 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * Trans_coord_5.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_6 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * Trans_coord_5.trans_coord.block<3,3>(0,0) * Trans_coord_6.trans_coord.block<3,3>(0,0) * Trans_coord_8.trans_coord.block<3,3>(0,0) * Trans_coord_9.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d p_base_Y = z_base_Y.cross(p_base_Y_E);

          Vector3d p_base_x = z_base_x;

          Vector3d p_1 = z_1.cross(p_1_E);

          Vector3d p_2 = z_2.cross(p_2_E);

          Vector3d p_3 = z_3.cross(p_3_E);

          Vector3d p_4 = z_4.cross(p_4_E);

          Vector3d p_5 = z_5.cross(p_5_E);

          Vector3d p_6 = z_6.cross(p_6_E);

          Matrix<double, 3, 8> J_pos;
          J_pos.col(0) = p_1;
          J_pos.col(1) = p_2;
          J_pos.col(2) = p_3; 
          J_pos.col(3) = p_4;
          J_pos.col(4) = p_5;
          J_pos.col(5) = p_6;
          J_pos.col(6) = p_base_Y;
          J_pos.col(7) = p_base_x;


          Matrix<double, 3, 8> J_rot;
          J_rot.col(0) = z_1;
          J_rot.col(1) = z_2;
          J_rot.col(2) = z_3;
          J_rot.col(3) = z_4;
          J_rot.col(4) = z_5;
          J_rot.col(5) = z_6;
          J_rot.col(6) = z_base_Y;
          J_rot.col(7) = Vector3d::Zero();

          Matrix<double, 6, 8> J_prot;
          J << J_pos, J_rot;

    return J;
}

//ヌル空間射影オペレータを計算する関数
void JointActionServer::cal_P(MatrixXd& P, const MatrixXd& I,const MatrixXd& J) {
    P = I - (J.transpose() * (J * J.transpose()).inverse() * J);
    return;
}

//勾配k(疑似行列の任意ベクトル)を計算する関数
void JointActionServer::cal_k(VectorXd& k, const MatrixXd& J, const VectorXd& e) {
    k = J.transpose() * W_E * e;
    return;
}

//
void JointActionServer::cal_ks(VectorXd& ks, VectorXd& q){
    ks = 1.0 * (q_d - q);
}

//減衰因子行列を計算する関数
void JointActionServer::cal_W_N(MatrixXd &W_N, const VectorXd &e) {
    W_N = 0.5 * (e.transpose() * W_E * e)(0,0) * MatrixXd::Identity(8,8)+W_N_bar;
    return;
}

void JointActionServer::cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N){
    H = J.transpose() * W_E * J + W_N;
    return;
}

} // namespace sobit_edu


