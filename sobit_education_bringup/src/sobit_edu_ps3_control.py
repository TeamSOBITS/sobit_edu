#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class Joy_control:
    def __init__(self):
        #subscriber
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.Joy_Callback, queue_size=1)
        self.sub_joint_state = rospy.Subscriber('/joint_states', JointState, self.Joint_state_Callback, queue_size=10)
        #publisher
        self.pub_joint_trajectory = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
        self.pub_twist = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
        #rate
        self.rate = rospy.Rate(5)
        #subscriberのメッセージを受け取る変数
        self.joint_state_msg = JointState()     
        self.joy_button = [0] * 17
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.3

    def Joint_state_Callback(self, msg):
        if msg.name[0] == "joint1":
            self.joint_state_msg = msg


    def Joy_Callback(self, msg):
        self.joy_button = msg.buttons
        self.left_joystick_lr = msg.axes[0] * self.magnifications
        self.left_joystick_ud = msg.axes[1] * self.magnifications
        self.right_joystick_lr = msg.axes[3] * self.magnifications
        self.right_joystick_ud = msg.axes[4] * self.magnifications
        

    def pub_joy(self):
        while(1):
            if self.joy_button[6] == True:#L2ボタンが押される
                rospy.loginfo("手先を操作するモード")
                self.move_joint("hand_motor_joint", self.left_joystick_ud + self.joint_state_msg.position[8])
            
            #カメラを動かす
            elif self.joy_button[7] == True:#R2ボタンが押される
                rospy.loginfo("カメラを動かすモード")
                self.move_joint("xtion_joint3", self.left_joystick_ud + self.joint_state_msg.position[9])
            
            #左手を動かす
            elif self.joy_button[4] == True:#L1ボタンが押される
                rospy.loginfo("腕を動かすモード")
                if self.joy_button[1] == True:#×ボタンが押される
                    self.move_joint("joint6", - self.left_joystick_ud + self.joint_state_msg.position[5])
                elif self.joy_button[2] == True:#○ボタンが押される
                    self.move_joint("joint4", - self.left_joystick_ud + self.joint_state_msg.position[3])
                elif self.joy_button[3] == True:#△ボタンが押される
                    self.move_joint("joint2", - self.left_joystick_ud + self.joint_state_msg.position[1])
                elif self.joy_button[0] == True:#□ボタンが押される
                    self.move_joint("joint1", self.left_joystick_ud + self.joint_state_msg.position[0])
            
            elif self.joy_button[12] == True:#スタートボタンを押す
                self.magnifications = 0.3
                self.move_joint("joint1", 0)
                self.move_joint("joint2", 0)
                self.move_joint("joint4", 1.31)
                self.move_joint("joint6", 0)
                self.move_joint("hand_motor_joint", 0)
                self.move_joint("xtion_joint3", -1.57)

            elif self.left_joystick_lr == 0 and self.left_joystick_ud == 0 and self.right_joystick_lr == 0 and self.right_joystick_ud == 0:
                pass
            #足を動かすモード
            else:
                rospy.loginfo("足を動かすモード")
                twist = Twist()
                twist.linear.x = self.left_joystick_ud * 0.2
                twist.angular.z = self.left_joystick_lr * 0.8
                self.pub_twist.publish(twist)
            rospy.Rate(10).sleep()

    def move_joint(self, joint_name, value):
        point = JointTrajectoryPoint()
        point.positions.append(value)

        send_data = JointTrajectory()
        send_data.joint_names.append(joint_name)
        send_data.points.append(point)
        #send_dataを送信
        self.pub_joint_trajectory.publish(send_data)




if __name__ == '__main__':
    rospy.init_node('sobit_edu_ps4_control_node')
    jc = Joy_control()
    jc.pub_joy();
    rospy.spin()
