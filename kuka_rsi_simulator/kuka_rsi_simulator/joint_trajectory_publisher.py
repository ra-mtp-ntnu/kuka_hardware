# Copyright 2019 Norwegian University of Science and Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.pub = self.create_publisher(
            JointTrajectory, '/my_robot_joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint_a1', 'joint_a2',
                           'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']

        ts = np.linspace(0.0, 100, 10000)
        for t in ts:
            point = JointTrajectoryPoint()
            point.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            point.positions = np.deg2rad(
                [(1.0 - np.cos(t)) * 90.0, -90.0, 90.0, 0.0, 45.0, 0.0]).tolist()
            point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.points.append(point)

        self.get_logger().info("publishing once")
        self.get_logger().info(str(msg))

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        joint_trajectory_publisher = JointTrajectoryPublisher()
        rclpy.spin_once(joint_trajectory_publisher)
    finally:
        joint_trajectory_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
