#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from joint_controller import JointController
from sensor_msgs.msg import JointState
from sobit_common_msg.srv import put_ctrl, put_ctrlResponse
from sobit_common_msg.msg import current_state, current_state_array
import tf
import math


class PutControl(JointController):

    def __init__(self):
        super(PutControl, self).__init__()
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.sub = rospy.Subscriber("/current_state_array", current_state_array, self.current_state_array_cb)
        self.pub_curr_ctrl = rospy.Publisher("/current_ctrl", current_state, queue_size=10)
        self.servise = rospy.Service("put_controller", put_ctrl, self.down_to_arm)
        self.joint_data = JointState()
        self.arm_flex_joint_current = 16140
    
    def current_state_array_cb(self, msg):
        for state in msg.current_state_array:
            if state.name == "arm_flex_joint":
                self.arm_flex_joint_current = state.current_ma

    def joint_states_callback(self, msg):
        if len(msg.name) == 7:
            self.joint_data = msg

    def down_to_arm(self, msg):
        fixed_x_cm = 14.5
        now = rospy.Time.now()
        rate = rospy.Rate(5)
        current_data = current_state()
        current_data.joint_name = "hand_motor_joint"
        current_data.current_ma = 403.5
        self.pub_curr_ctrl.publish(current_data)
        try:
            self.listener.waitForTransform("base_footprint", "hand_motor_link", now, rospy.Duration(1.0))
            (trans, _) = self.listener.lookupTransform("base_footprint", "hand_motor_link", now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf transform lookup failed")
            return put_ctrlResponse(False)
        except Exception as e:
            rospy.logerr(e)
            return put_ctrlResponse(False)
        target_z_cm = trans[2] * 100 - self.from_base_to_arm_flex_link_z_cm
        target_x_cm = fixed_x_cm
        arm_flex_link_cm = math.sqrt(self.arm_flex_link_x_cm**2 + self.arm_flex_link_z_cm**2)
        while target_z_cm > -10.0:
            target_z_cm -= 0.5
            if self.arm_flex_joint_current < 16140:
                for i, joint_name in enumerate(self.joint_data.name):
                    if joint_name == "arm_flex_joint":
                        self.add_arm_control_data_to_storage("arm_flex_joint", self.joint_data.position[i])
                    elif joint_name == "arm_roll_joint":
                        self.add_arm_control_data_to_storage("arm_roll_joint", self.joint_data.position[i])
                    elif joint_name == "elbow_flex_joint":
                        self.add_arm_control_data_to_storage("elbow_flex_joint", self.joint_data.position[i])
                    elif joint_name == "wrist_flex_joint":
                        self.add_arm_control_data_to_storage("wrist_flex_joint", self.joint_data.position[i])
                self.publish_arm_control_data(0.001)
                break
            target_rad = math.pi / 2. - math.atan2(target_z_cm, target_x_cm)
            target_dis_cm = math.sqrt(target_x_cm**2 + target_z_cm**2)
            tmp_arm_flex_joint_rad = math.acos(
                (target_dis_cm**2 + arm_flex_link_cm**2 - self.wrist_flex_link_z_cm**2) / (2. * target_dis_cm * arm_flex_link_cm))
            arm_flex_joint_rad = target_rad - tmp_arm_flex_joint_rad - math.acos(self.arm_flex_link_z_cm / arm_flex_link_cm)
            tmp_elbow_flex_joint_rad = math.acos(
                (self.wrist_flex_link_z_cm**2 + arm_flex_link_cm**2 - target_dis_cm**2) / (2. * self.wrist_flex_link_z_cm * arm_flex_link_cm))
            elbow_flex_joint_rad = -(tmp_elbow_flex_joint_rad - (math.pi - math.atan(self.arm_flex_link_z_cm / self.arm_flex_link_x_cm)))
            wrist_flex_joint_rad = math.pi / 2. - (arm_flex_joint_rad + math.pi / 2 + elbow_flex_joint_rad)
            self.add_arm_control_data_to_storage("arm_flex_joint", arm_flex_joint_rad)
            self.add_arm_control_data_to_storage("elbow_flex_joint", elbow_flex_joint_rad)
            self.add_arm_control_data_to_storage("wrist_flex_joint", wrist_flex_joint_rad)
            self.publish_arm_control_data(0.0001)
            rate.sleep()

        return put_ctrlResponse(True)


if __name__ == "__main__":
    rospy.init_node("put_controller_node")
    pc = PutControl()
    rospy.spin()