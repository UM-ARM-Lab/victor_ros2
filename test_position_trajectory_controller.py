#!/usr/bin/env python3
"""
Use this to test that the position_trajectory_controller works as expected.
It will move the final joint on the left arm (left_joint_a7) from it's current position,
to 0, then to 1, then back to 0.
"""

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")

        wait_sec_between_publish = 15
        self.test_joint = "left_joint_a7"

        self.traj = JointTrajectory()
        self.traj.joint_names = [self.test_joint]
        self.traj.points = [
            JointTrajectoryPoint(
                time_from_start=Duration(sec=1),
                positions=[0.,],
                velocities=[0.,],
            ),
            JointTrajectoryPoint(
                time_from_start=Duration(sec=5),
                positions=[1.,],
                velocities=[0.,],
            ),
            JointTrajectoryPoint(
                time_from_start=Duration(sec=10),
                positions=[0.,],
                velocities=[0.,],
            ),
        ]

        publish_topic = "/position_trajectory_controller/joint_trajectory"

        self.get_logger().info(f'Publishing on topic "{publish_topic}" every {wait_sec_between_publish} s')

        self.starting_msg = None

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)

    def timer_callback(self):
        if self.starting_msg is not None:
            self.get_logger().info(f"Sending goal")
            self.publisher_.publish(self.traj)

    def joint_state_callback(self, msg):
        if self.starting_msg is None:
            self.starting_msg = msg
            initial_pos = msg.position[msg.name.index(self.test_joint)]

            self.traj.points.insert(0, JointTrajectoryPoint(
                time_from_start=Duration(sec=1),
                positions=[initial_pos,],
                velocities=[0.,],
            ))


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
