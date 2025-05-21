#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf_transformations import euler_from_quaternion
import math

class JointTrajectoryConverter(Node):
    def __init__(self):
        super().__init__('joint_trajectory_converter')

        # Publishers and Subscribers
        self.traj_publisher = self.create_publisher(
            JointTrajectory, '/go2_controller/joint_trajectory', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.body_pose_sub = self.create_subscription(
            Pose, '/body_pose', self.body_pose_callback, 10)

        # Joint names from controllers.yaml
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]

        # Gait parameters
        self.gait_frequency = 2.0  # Hz
        self.gait_amplitude = 0.3  # Radians for thigh/calf swing
        self.hip_swing = 0.1  # Radians for hip
        self.time = 0.0
        self.dt = 0.01  # Control period

        # Body pose
        self.body_height = 0.3
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Velocity
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Timer for gait generation
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Joint Trajectory Converter started')

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        self.get_logger().info(
            f'Velocity: x={self.linear_x:.2f}, y={self.linear_y:.2f}, z={self.angular_z:.2f}'
        )

    def body_pose_callback(self, msg):
        self.body_height = msg.position.z
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        self.get_logger().info(
            f'Body pose: height={self.body_height:.2f}, '
            f'roll={self.roll:.2f}, pitch={self.pitch:.2f}, yaw={self.yaw:.2f}'
        )

    def timer_callback(self):
        self.time += self.dt
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        # Nominal joint positions (standing pose)
        positions = [0.0] * 12
        thigh_neutral = -1.0
        calf_neutral = -1.5
        positions[1] = thigh_neutral  # FL_thigh
        positions[2] = calf_neutral   # FL_calf
        positions[4] = thigh_neutral  # FR_thigh
        positions[5] = calf_neutral   # FR_calf
        positions[7] = thigh_neutral  # RL_thigh
        positions[8] = calf_neutral   # RL_calf
        positions[10] = thigh_neutral # RR_thigh
        positions[11] = calf_neutral  # RR_calf

        # Compute speed and direction
        speed = math.sqrt(self.linear_x**2 + self.linear_y**2)
        speed_factor = speed / 0.5 if speed != 0 else 1.0
        direction = math.atan2(self.linear_y, self.linear_x) if speed != 0 else 0.0
        phase = 2 * math.pi * self.gait_frequency * self.time

        # Trot gait for linear motion (forward/backward, left/right)
        if speed > 0:
            phase_shift = direction * 0.5  # Adjust gait for direction
            positions[1] += self.gait_amplitude * math.sin(phase + phase_shift) * speed_factor  # FL_thigh
            positions[2] += self.gait_amplitude * math.cos(phase + phase_shift) * speed_factor  # FL_calf
            positions[10] += self.gait_amplitude * math.sin(phase + phase_shift) * speed_factor # RR_thigh
            positions[11] += self.gait_amplitude * math.cos(phase + phase_shift) * speed_factor # RR_calf
            positions[4] += self.gait_amplitude * math.sin(phase + math.pi - phase_shift) * speed_factor  # FR_thigh
            positions[5] += self.gait_amplitude * math.cos(phase + math.pi - phase_shift) * speed_factor  # FR_calf
            positions[7] += self.gait_amplitude * math.sin(phase + math.pi - phase_shift) * speed_factor  # RL_thigh
            positions[8] += self.gait_amplitude * math.cos(phase + math.pi - phase_shift) * speed_factor  # RL_calf
            positions[0] += self.hip_swing * math.sin(phase) * speed_factor  # FL_hip
            positions[3] += -self.hip_swing * math.sin(phase) * speed_factor # FR_hip
            positions[6] += self.hip_swing * math.sin(phase) * speed_factor  # RL_hip
            positions[9] += -self.hip_swing * math.sin(phase) * speed_factor # RR_hip

        # Turning (angular.z)
        if self.angular_z != 0:
            turn_factor = abs(self.angular_z) / 1.0
            # Adjust hip joints for turning
            turn_sign = 1 if self.angular_z > 0 else -1
            positions[0] += self.hip_swing * turn_factor * turn_sign  # FL_hip
            positions[3] -= self.hip_swing * turn_factor * turn_sign  # FR_hip
            positions[6] -= self.hip_swing * turn_factor * turn_sign  # RL_hip
            positions[9] += self.hip_swing * turn_factor * turn_sign  # RR_hip
            # Adjust thigh joints for turning
            positions[1] += self.gait_amplitude * 0.2 * turn_factor * turn_sign  # FL_thigh
            positions[4] -= self.gait_amplitude * 0.2 * turn_factor * turn_sign  # FR_thigh
            positions[7] -= self.gait_amplitude * 0.2 * turn_factor * turn_sign  # RL_thigh
            positions[10] += self.gait_amplitude * 0.2 * turn_factor * turn_sign # RR_thigh

        # Body pose adjustments
        height_adjust = (self.body_height - 0.3) * 2.0
        positions[1] += height_adjust
        positions[2] += height_adjust
        positions[4] += height_adjust
        positions[5] += height_adjust
        positions[7] += height_adjust
        positions[8] += height_adjust
        positions[10] += height_adjust
        positions[11] += height_adjust

        roll_adjust = self.roll * 0.5
        pitch_adjust = self.pitch * 0.5
        positions[0] += roll_adjust   # FL_hip
        positions[3] -= roll_adjust   # FR_hip
        positions[6] -= roll_adjust   # RL_hip
        positions[9] += roll_adjust   # RR_hip
        positions[1] += pitch_adjust  # FL_thigh
        positions[4] += pitch_adjust  # FR_thigh
        positions[7] -= pitch_adjust  # RL_thigh
        positions[10] -= pitch_adjust # RR_thigh

        point.positions = positions
        traj.points = [point]
        self.traj_publisher.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()