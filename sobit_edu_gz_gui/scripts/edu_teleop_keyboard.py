#!/usr/bin/env python3
# Keyboard teleop for SOBIT EDU, modeled on gz_human_sim's
# human_teleop_switcher.py so the same key layout works for both. Publishes
# straight to the ROS topic robot.launch.py's twist_stamper reads
# (/<robot_name>/commands/velocity), so it can run alongside EduRobotManager's
# GUI slider/buttons (which drive the same robot through the gz-side
# /<robot_name>/cmd_vel bridge) without conflict -- whichever one last
# published simply wins for that tick, same as any two cmd_vel sources.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty


class EduTeleopKeyboard(Node):

    def __init__(self):
        super().__init__('edu_teleop_keyboard')

        self.declare_parameter('robot_name', 'sobit_edu')
        robot_name = self.get_parameter('robot_name').value

        self.pub = self.create_publisher(
            Twist,
            f'/{robot_name}/commands/velocity',
            10
        )

        self.linear_speed = 0.3
        self.angular_speed = 0.5

        self.speed_step = 1.1

        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0

        self.min_linear_speed = 0.05
        self.min_angular_speed = 0.05

        self.settings = termios.tcgetattr(sys.stdin)

        self.print_help(robot_name)

    def print_help(self, robot_name):
        msg = """

------------------------
SOBIT EDU Teleop Keyboard
------------------------

Moving around:

   u    i    o
   j    k    l
   m    ,    .

anything else : stop


Speed control:

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%


CTRL-C to quit

currently:
    target : {}
    speed  : {:.2f}
    turn   : {:.2f}

------------------------

""".format(
            robot_name,
            self.linear_speed,
            self.angular_speed
        )

        print(msg)

    def publish_twist(self, msg):
        self.pub.publish(msg)

    def keyboard_loop(self):
        key = self.get_key()

        if key == 'q':
            self.linear_speed *= self.speed_step
            self.angular_speed *= self.speed_step

            self.linear_speed = min(self.linear_speed, self.max_linear_speed)
            self.angular_speed = min(self.angular_speed, self.max_angular_speed)

            self.print_status()
            return

        if key == 'z':
            self.linear_speed /= self.speed_step
            self.angular_speed /= self.speed_step

            self.linear_speed = max(self.linear_speed, self.min_linear_speed)
            self.angular_speed = max(self.angular_speed, self.min_angular_speed)

            self.print_status()
            return

        if key == 'w':
            self.linear_speed *= self.speed_step
            self.linear_speed = min(self.linear_speed, self.max_linear_speed)
            self.print_status()
            return

        if key == 'x':
            self.linear_speed /= self.speed_step
            self.linear_speed = max(self.linear_speed, self.min_linear_speed)
            self.print_status()
            return

        if key == 'e':
            self.angular_speed *= self.speed_step
            self.angular_speed = min(self.angular_speed, self.max_angular_speed)
            self.print_status()
            return

        if key == 'c':
            self.angular_speed /= self.speed_step
            self.angular_speed = max(self.angular_speed, self.min_angular_speed)
            self.print_status()
            return

        twist = Twist()

        if key == 'i':
            twist.linear.x = self.linear_speed

        elif key == ',':
            twist.linear.x = -self.linear_speed

        elif key == 'j':
            twist.angular.z = self.angular_speed

        elif key == 'l':
            twist.angular.z = -self.angular_speed

        elif key == 'u':
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

        elif key == 'o':
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed

        elif key == 'm':
            twist.linear.x = -self.linear_speed
            twist.angular.z = self.angular_speed

        elif key == '.':
            twist.linear.x = -self.linear_speed
            twist.angular.z = -self.angular_speed

        elif key == 'k':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        else:
            return

        self.publish_twist(twist)

    def print_status(self):
        print(
            "\ncurrently:"
            f"  speed {self.linear_speed:.2f}"
            f"  turn {self.angular_speed:.2f}"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())

        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        return key


def main(args=None):
    rclpy.init(args=args)

    node = EduTeleopKeyboard()

    try:
        while rclpy.ok():
            node.keyboard_loop()
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
