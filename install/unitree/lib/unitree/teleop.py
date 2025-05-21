#!/usr/bin/env python3

from __future__ import print_function

import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
from rclpy.node import Node

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Teleop(Node):
    def __init__(self):
        super().__init__('unitree_teleop')

        # Publishers for velocity and pose
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher = self.create_publisher(Pose, 'body_pose', 10)

        # Subscriber for joystick input
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Declare parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('swing_height', 0.0)
        self.declare_parameter('nominal_height', 0.0)

        # Get parameter values
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.swing_height = self.get_parameter('swing_height').value
        self.nominal_height = self.get_parameter('nominal_height').value

        # Keyboard control message
        self.msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
U    I    O
J    K    L
M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
        """

        # Velocity bindings for keyboard control
        self.velocityBindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, -1),
            'j': (0, 0, 0, 1),
            'l': (0, 0, 0, -1),
            'u': (1, 0, 0, 1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, 1),
            'm': (-1, 0, 0, -1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, 1, 0),
            'b': (0, 0, -1, 0),
        }

        # Pose bindings for keyboard control
        self.poseBindings = {
            'f': (-1, 0, 0, 0),
            'h': (1, 0, 0, 0),
            't': (0, 1, 0, 0),
            'b': (0, -1, 0, 0),
            'r': (0, 0, 1, 0),
            'y': (0, 0, -1, 0),
        }

        # Speed adjustment bindings
        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (0.9, 0.9),
            'w': (1.1, 1),
            'x': (0.9, 1),
            'e': (1, 1.1),
            'c': (1, 0.9),
        }

        # Start polling keys
        self.poll_keys()

    def joy_callback(self, data):
        # Handle joystick input
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        # Publish pose from joystick input
        body_pose = Pose()
        body_pose.position.z = data.axes[5] * 0.5 if data.axes[5] < 0 else 0.0

        roll = (not data.buttons[5]) * -data.axes[3] * 0.349066
        pitch = data.axes[4] * 0.174533
        yaw = data.buttons[5] * data.axes[3] * 0.436332

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(body_pose)

    def poll_keys(self):
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        x = 0
        y = 0
        z = 0
        th = 0
        status = 0
        cmd_attempts = 0

        try:
            print(self.msg)
            print(self.vels(self.speed, self.turn))

            while rclpy.ok():
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]

                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x * self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist)

                    cmd_attempts += 1

                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]

                    print(self.vels(self.speed, self.turn))
                    if status == 14:
                        print(self.msg)
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0
                    if key == '\x03':  # CTRL-C
                        break

        except Exception as e:
            print(e)

        finally:
            # Publish zero velocity on exit
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed}\tturn {turn}"

if __name__ == '__main__':
    rclpy.init()
    teleop = Teleop()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()