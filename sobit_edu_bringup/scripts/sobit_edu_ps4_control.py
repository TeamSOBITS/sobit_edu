#!/usr/bin/env python3
# coding: utf-8
import rospy

from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class JoyControl:
    def __init__(self):
        # Subscriber
        self.sub_joy         = rospy.Subscriber('/joy', Joy, self.Joy_Callback, queue_size=1)
        self.sub_joint_state = rospy.Subscriber('/joint_states', JointState, self.Joint_state_Callback, queue_size=10)

        # Publisher
        self.pub_joint_trajectory = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
        self.pub_twist            = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)

        # Parameters
        self.joint_state_msg   = JointState()     
        self.joy_button        = [0] * 17
        self.left_joystick_lr  = 0
        self.left_joystick_ud  = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications    = 0.3

        # Other buttons
        """
        self.cross_button = False
        self.circle_button = False
        self.triangle_button = False
        self.square_button = False
        
        self.l1_button = False
        self.r1_button = False
        self.l2_button = False
        self.r2_button = False

        self.select_button = False
        self.start_button = False
        self.home_button = False

        self.l3_button = False
        self.r3_button = False

        self.top_arrow_button = False
        self.bottom_arrow_button = False
        self.left_arrow_button = False
        self.right_arrow_button = False
        """


    def Joint_state_Callback(self, msg):
        if msg.name:
            self.joint_state_msg = msg


    def Joy_Callback(self, msg):
        self.joy_button        = msg.buttons
        self.left_joystick_lr  = msg.axes[0] * self.magnifications
        self.left_joystick_ud  = msg.axes[1] * self.magnifications
        self.right_joystick_lr = msg.axes[3] * self.magnifications
        self.right_joystick_ud = msg.axes[4] * self.magnifications

        # L2 button pressed: Move the end effector
        if self.joy_button[6]:
            rospy.loginfo("Mode: end effector control")

            self.move_joint("hand_joint", self.left_joystick_ud + self.joint_state_msg.position[8])
        
        # R2 button pressed: Move the camera
        elif self.joy_button[7]:
            rospy.loginfo("Mode: camera control")

            if self.joy_button[1]:
                self.move_joint("head_camera_pan_joint" ,   self.left_joystick_ud + self.joint_state_msg.position[9])

            elif self.joy_button[2]:
                self.move_joint("head_camera_tilt_joint", - self.left_joystick_ud + self.joint_state_msg.position[10])

        # L1 button pressed: Move the arm
        elif self.joy_button[4]:
            rospy.loginfo("Mode: arm control")

            # x button pressed
            if self.joy_button[1]:
                self.move_joint("arm_wrist_tilt_joint"     , - self.left_joystick_ud + self.joint_state_msg.position[5])
            
            # circle button pressed
            elif self.joy_button[2]:
                self.move_joint("arm_elbow_1_tilt_joint"   , - self.left_joystick_ud + self.joint_state_msg.position[3])
            
            # triangle button pressed
            elif self.joy_button[3]:
                self.move_joint("arm_shoulder_1_tilt_joint", - self.left_joystick_ud + self.joint_state_msg.position[1])
            
            # square button pressed
            elif self.joy_button[0]:
                self.move_joint("arm_shoulder_pan_joint"   ,   self.left_joystick_ud + self.joint_state_msg.position[0])
        
        # Start button pressed: Initialize the robot
        elif self.joy_button[12]:
            self.magnifications = 0.3

            self.move_joint("head_camera_pan_joint"    , 0.0)
            self.move_joint("head_camera_tilt_joint"   , 0.0)
            self.move_joint("arm_shoulder_pan_joint"   , 0.0)
            self.move_joint("arm_shoulder_1_tilt_joint", 0.0)
            self.move_joint("arm_elbow_1_tilt_joint"   , 0.0)
            self.move_joint("arm_wrist_tilt_joint"     , 0.0)
            self.move_joint("hand_motor_joint"         , 0.0)
            self.move_joint("hand_joint"               , 0.0)

        # elif self.left_joystick_lr == 0 and self.left_joystick_ud == 0 and self.right_joystick_lr == 0 and self.right_joystick_ud == 0:
        #     pass

        # Move the legs
        else:
            rospy.loginfo("Mode: leg control")

            twist = Twist()
            twist.linear.x = self.left_joystick_ud * 0.2
            twist.angular.z = self.left_joystick_lr * 0.8

            self.pub_twist.publish(twist)


    def move_joint(self, joint_name, value):
        point = JointTrajectoryPoint()
        point.positions.append(value)

        send_data = JointTrajectory()
        send_data.joint_names.append(joint_name)
        send_data.points.append(point)

        self.pub_joint_trajectory.publish(send_data)


if __name__ == '__main__':
    rospy.init_node('sobit_edu_ps4_control_node')
    jc = JoyControl()
    rospy.spin()
