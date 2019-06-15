#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sobit_common_msg.srv import gripper_ctrl
from sobit_common_msg.srv import gripper_ctrlResponse
from sobit_common_msg.srv import gripper_move
from sobit_common_msg.srv import gripper_moveResponse
from sobit_common_msg.srv import robot_motion
from sobit_common_msg.srv import robot_motionResponse
from sobit_common_msg.srv import odom_base


class JointController:

    def __init__(self):
        self.pub_arm_control = rospy.Publisher("/arm_trajectory_controller/command", JointTrajectory, queue_size=10)
        self.pub_xtion_control = rospy.Publisher("/xtion_trajectory_controller/command", JointTrajectory, queue_size=10)
        self.servise = rospy.Service("gripper_open_and_close", gripper_ctrl, self.open_and_close_gripper_server)
        self.servise = rospy.Service("gripper_move_to_target", gripper_move, self.move_gripper_to_target_server)
        self.servise = rospy.Service("motion_ctrl", robot_motion, self.move_to_registered_motion_server)
        self.listener = tf.TransformListener()
        self.arm_control_data = JointTrajectory()
        self.xtion_control_data = JointTrajectory()
        self.arm_control_data.points = [JointTrajectoryPoint()]
        self.xtion_control_data.points = [JointTrajectoryPoint()]
        self.from_base_to_arm_flex_link_x_cm = 12.2
        self.from_base_to_arm_flex_link_z_cm = 52.2
        self.arm_flex_link_x_cm = 2.4
        self.arm_flex_link_z_cm = 14.8
        self.wrist_flex_link_z_cm = 15.0
        self.can_grasp_min_z_cm = 35.0
        self.can_grasp_max_z_cm = 80.0

    def move_gripper_to_target_server(self, req_msg):
        """
            hand_motor_link を指定したtfに動かすサービス
            target_name : 目標のtf名
            shift       : 目標のtfの位置からどれだけずらすか
        """
        target_object = req_msg.target_name
        key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))  # 座標変換の可否判定
        if not key:
            rospy.logerr("gripper_move_to_target Can't Transform [%s]", target_object)
            return gripper_moveResponse(False)

        rospy.loginfo("Gripper_move_to_target Target_object [%s]", target_object)
        (trans, _) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))

        tan_rad = math.atan((trans[1] + req_msg.shift.y) / trans[0])
        sx = math.cos(tan_rad) * req_msg.shift.x
        sy = math.sin(tan_rad) * req_msg.shift.x
        object_x_cm = (trans[0] + sx) * 100
        object_y_cm = (trans[1] + sy) * 100
        object_z_cm = (trans[2] + req_msg.shift.z) * 100
        if object_z_cm < self.can_grasp_min_z_cm or object_z_cm > self.can_grasp_max_z_cm:
            return gripper_moveResponse(False)
        arm_flex_joint_rad = 0
        elbow_flex_joint_rad = 0
        wrist_flex_joint_rad = 0
        from_base_to_hand_motor_link_x_cm = 0
        if self.from_base_to_arm_flex_link_z_cm < object_z_cm < self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm:
            print "objectがelbow_flex_jointより低い場合"
            elbow_flex_joint_sin = (self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm -
                                    object_z_cm) / self.wrist_flex_link_z_cm
            elbow_flex_joint_rad = math.asin(elbow_flex_joint_sin)
            wrist_flex_joint_rad = -elbow_flex_joint_rad
            arm_flex_joint_rad = 0.0
            from_base_to_hand_motor_link_x_cm = self.from_base_to_arm_flex_link_x_cm + self.arm_flex_link_x_cm + self.wrist_flex_link_z_cm * math.cos(
                elbow_flex_joint_rad)

        elif self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm < object_z_cm:
            print "objectがelbow_flex_jointより高い場合"
            elbow_flex_joint_sin = (
                object_z_cm -
                (self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm)) / self.wrist_flex_link_z_cm
            elbow_flex_joint_rad = -math.asin(elbow_flex_joint_sin)
            wrist_flex_joint_rad = -elbow_flex_joint_rad
            arm_flex_joint_rad = 0.0
            from_base_to_hand_motor_link_x_cm = self.from_base_to_arm_flex_link_x_cm + self.arm_flex_link_x_cm + self.wrist_flex_link_z_cm * math.cos(
                elbow_flex_joint_rad)

        elif object_z_cm < self.from_base_to_arm_flex_link_z_cm:
            print "objectがarm_flex_jointより低い場合"
            elbow_flex_joint_sin = (self.from_base_to_arm_flex_link_z_cm - self.arm_flex_link_x_cm -
                                    object_z_cm) / self.wrist_flex_link_z_cm
            elbow_flex_joint_rad = -math.asin(elbow_flex_joint_sin)
            wrist_flex_joint_rad = -elbow_flex_joint_rad
            arm_flex_joint_rad = 1.57
            from_base_to_hand_motor_link_x_cm = self.from_base_to_arm_flex_link_x_cm + self.arm_flex_link_z_cm + self.wrist_flex_link_z_cm * math.cos(
                elbow_flex_joint_rad)

        time_from_start_sec = 0.3
        self.add_arm_control_data_to_storage("arm_roll_joint", 0.0)
        self.add_arm_control_data_to_storage("arm_flex_joint", arm_flex_joint_rad)
        self.add_arm_control_data_to_storage("elbow_flex_joint", elbow_flex_joint_rad)
        self.add_arm_control_data_to_storage("wrist_flex_joint", wrist_flex_joint_rad)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(2.0)

        turning_deg = math.degrees(math.atan(object_y_cm / object_x_cm))
        str_turning_deg = "T:" + str(turning_deg)
        self.move_wheel(str_turning_deg)
        rospy.sleep(2)

        moving_cm = math.sqrt(object_x_cm**2 + object_y_cm**2) - from_base_to_hand_motor_link_x_cm
        str_moving_cm = "S:" + str(moving_cm)
        self.move_wheel(str_moving_cm)

        return gripper_moveResponse(True)

    def open_and_close_gripper_server(self, req_msg):
        hand_motor_joint_rad = req_msg.rad
        time_from_start_sec = 0.1
        self.add_arm_control_data_to_storage("hand_motor_joint", hand_motor_joint_rad)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(time_from_start_sec)
        return gripper_ctrlResponse(True)

    def move_to_registered_motion_server(self, req_msg):
        motion_type = req_msg.motion_type
        if motion_type == "INITIAL_POSE":
            self.move_to_initial_pose()
        elif motion_type == "DETECTING_POSE":
            self.move_to_detecting_pose()
        return robot_motionResponse(True)

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def add_arm_control_data_to_storage(self, joint_name, rad):
        self.arm_control_data.joint_names.append(joint_name)
        self.arm_control_data.points[0].positions.append(rad)
        self.arm_control_data.points[0].velocities.append(0)
        self.arm_control_data.points[0].accelerations.append(0)
        self.arm_control_data.points[0].effort.append(0)

    def add_xtion_control_data_to_storage(self, joint_name, rad):
        self.xtion_control_data.joint_names.append(joint_name)
        self.xtion_control_data.points[0].positions.append(rad)
        self.xtion_control_data.points[0].velocities.append(0)
        self.xtion_control_data.points[0].accelerations.append(0)
        self.xtion_control_data.points[0].effort.append(0)

    def publish_arm_control_data(self, time_from_start_sec):
        self.arm_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_arm_control)
        self.pub_arm_control.publish(self.arm_control_data)
        self.arm_control_data = JointTrajectory()
        self.arm_control_data.points = [JointTrajectoryPoint()]

    def publish_xtion_control_data(self, time_from_start_sec):
        self.xtion_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_xtion_control)
        self.pub_xtion_control.publish(self.xtion_control_data)
        self.xtion_control_data = JointTrajectory()
        self.xtion_control_data.points = [JointTrajectoryPoint()]

    def move_wheel(self, str_distance):
        rospy.wait_for_service('/robot_ctrl/odom_base_ctrl')
        try:
            wheel_ctrl_service = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
            res = wheel_ctrl_service(str_distance)
            return res.res_str
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e

    def move_to_initial_pose(self):
        time_from_start_sec = 0.5
        self.add_arm_control_data_to_storage("arm_roll_joint", 0.00)
        self.add_arm_control_data_to_storage("arm_flex_joint", -1.57)
        self.add_arm_control_data_to_storage("elbow_flex_joint", 1.32)
        self.add_arm_control_data_to_storage("wrist_flex_joint", 0.25)
        self.add_arm_control_data_to_storage("hand_motor_joint", 0.00)
        self.add_xtion_control_data_to_storage("xtion_tilt_joint", 0.00)
        self.add_xtion_control_data_to_storage("xtion_pan_joint", 0.00)
        self.publish_arm_control_data(time_from_start_sec)
        self.publish_xtion_control_data(time_from_start_sec)
        rospy.sleep(1.0)

    def move_to_detecting_pose(self):
        time_from_start = 0.5
        self.add_xtion_control_data_to_storage("xtion_tilt_joint", 0.53)
        self.publish_xtion_control_data(time_from_start)
        rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("joint_controller")
    jc = JointController()
    jc.move_to_initial_pose()
    rospy.spin()
