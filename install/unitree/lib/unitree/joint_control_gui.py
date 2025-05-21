#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tkinter as tk
from threading import Lock
import numpy as np

class JointControlGUI(Node):
    def __init__(self):
        super().__init__('joint_control_gui')

        # Joint names and limits from URDF
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        self.joint_limits = {
            'FL_hip_joint': (-1.0472, 1.0472),
            'FL_thigh_joint': (-1.5708, 3.4907),
            'FL_calf_joint': (-2.7227, -0.83776),
            'FR_hip_joint': (-1.0472, 1.0472),
            'FR_thigh_joint': (-1.5708, 3.4907),
            'FR_calf_joint': (-2.7227, -0.83776),
            'RL_hip_joint': (-1.0472, 1.0472),
            'RL_thigh_joint': (-0.5236, 4.5379),
            'RL_calf_joint': (-2.7227, -0.83776),
            'RR_hip_joint': (-1.0472, 1.0472),
            'RR_thigh_joint': (-0.5236, 4.5379),
            'RR_calf_joint': (-2.7227, -0.83776)
        }

        # Publisher for joint trajectory commands
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/go2_description/go2_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/go2_description/joint_states',
            self.joint_state_callback,
            10
        )

        # Lock for thread-safe GUI updates
        self.lock = Lock()

        # Current joint positions (initialized to 0.0)
        self.current_positions = {name: 0.0 for name in self.joint_names}

        # Setup GUI
        self.root = tk.Tk()
        self.root.title('Unitree Go2 Joint Control')
        self.sliders = {}
        self.labels = {}

        # Create sliders and labels for each joint
        for i, joint in enumerate(self.joint_names):
            # Label for joint name and current position
            label = tk.Label(
                self.root,
                text=f"{joint}: {self.current_positions[joint]:.3f} rad"
            )
            label.grid(row=i, column=0, padx=5, pady=2, sticky='w')
            self.labels[joint] = label

            # Slider for joint position
            slider = tk.Scale(
                self.root,
                from_=self.joint_limits[joint][0],
                to=self.joint_limits[joint][1],
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=300,
                command=lambda val, j=joint: self.slider_callback(j, float(val))
            )
            slider.set(self.current_positions[joint])
            slider.grid(row=i, column=1, padx=5, pady=2)
            self.sliders[joint] = slider

        # Start GUI update loop
        self.root.after(100, self.update_gui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def slider_callback(self, joint_name, value):
        """Publish JointTrajectory when a slider is moved."""
        with self.lock:
            self.current_positions[joint_name] = value
            self.labels[joint_name].config(
                text=f"{joint_name}: {value:.3f} rad"
            )

        # Create and publish trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [value]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1s
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

    def joint_state_callback(self, msg):
        """Update current positions from joint states."""
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.current_positions:
                    self.current_positions[name] = pos

    def update_gui(self):
        """Update sliders and labels with current joint positions."""
        with self.lock:
            for joint in self.joint_names:
                pos = self.current_positions[joint]
                self.sliders[joint].set(pos)
                self.labels[joint].config(text=f"{joint}: {pos:.3f} rad")

        self.root.after(100, self.update_gui)

    def on_closing(self):
        """Cleanup on window close."""
        # Publish zero positions to stop the robot
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

        self.root.destroy()
        rclpy.shutdown()

    def run(self):
        """Run the node and GUI."""
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.mainloop()

def main():
    rclpy.init()
    node = JointControlGUI()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_closing()

if __name__ == '__main__':
    main()