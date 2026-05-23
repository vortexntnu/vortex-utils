#!/usr/bin/env python3
"""Send the same waypoint goal to both the RPY and quaternion reference filter
action servers simultaneously so their outputs can be compared."""

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import ReferenceFilterQuatWaypoint, ReferenceFilterWaypoint
from vortex_utils.python_utils import euler_to_quat


class DualGoalSender(Node):
    def __init__(self, x, y, z, roll, pitch, yaw, ns, threshold):
        super().__init__('dual_goal_sender')

        self._rpy_client = ActionClient(
            self, ReferenceFilterWaypoint, f'/{ns}/reference_filter'
        )
        self._quat_client = ActionClient(
            self, ReferenceFilterQuatWaypoint, f'/{ns}/reference_filter_quat'
        )

        self._rpy_done = False
        self._quat_done = False

        quat = euler_to_quat(roll=roll, pitch=pitch, yaw=yaw)

        # Build RPY goal
        rpy_goal = ReferenceFilterWaypoint.Goal()
        rpy_goal.waypoint.pose.position.x = x
        rpy_goal.waypoint.pose.position.y = y
        rpy_goal.waypoint.pose.position.z = z
        rpy_goal.waypoint.pose.orientation.x = quat[0]
        rpy_goal.waypoint.pose.orientation.y = quat[1]
        rpy_goal.waypoint.pose.orientation.z = quat[2]
        rpy_goal.waypoint.pose.orientation.w = quat[3]
        rpy_goal.convergence_threshold = threshold

        # Build quaternion goal
        quat_goal = ReferenceFilterQuatWaypoint.Goal()
        quat_goal.waypoint.pose.position.x = x
        quat_goal.waypoint.pose.position.y = y
        quat_goal.waypoint.pose.position.z = z
        quat_goal.waypoint.pose.orientation.x = quat[0]
        quat_goal.waypoint.pose.orientation.y = quat[1]
        quat_goal.waypoint.pose.orientation.z = quat[2]
        quat_goal.waypoint.pose.orientation.w = quat[3]
        quat_goal.convergence_threshold = threshold

        self.get_logger().info(
            f'Goal: pos=({x}, {y}, {z}) rpy=({roll}, {pitch}, {yaw})'
        )

        # Wait for both servers
        self.get_logger().info('Waiting for RPY action server...')
        self._rpy_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Waiting for Quat action server...')
        self._quat_client.wait_for_server(timeout_sec=10.0)

        # Send both goals
        self.get_logger().info('Sending goals to both servers...')
        rpy_future = self._rpy_client.send_goal_async(rpy_goal)
        rpy_future.add_done_callback(self._rpy_response_cb)

        quat_future = self._quat_client.send_goal_async(quat_goal)
        quat_future.add_done_callback(self._quat_response_cb)

    def _rpy_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('RPY goal rejected')
            self._rpy_done = True
            self._maybe_shutdown()
            return
        self.get_logger().info('RPY goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rpy_result_cb)

    def _quat_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Quat goal rejected')
            self._quat_done = True
            self._maybe_shutdown()
            return
        self.get_logger().info('Quat goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._quat_result_cb)

    def _rpy_result_cb(self, future):
        result = future.result().result.success
        self.get_logger().info(f'RPY result: {result}')
        self._rpy_done = True
        self._maybe_shutdown()

    def _quat_result_cb(self, future):
        result = future.result().result.success
        self.get_logger().info(f'Quat result: {result}')
        self._quat_done = True
        self._maybe_shutdown()

    def _maybe_shutdown(self):
        if self._rpy_done and self._quat_done:
            self.get_logger().info('Both goals finished')
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Send dual reference filter goals')
    parser.add_argument('-x', type=float, default=5.0)
    parser.add_argument('-y', type=float, default=3.0)
    parser.add_argument('-z', type=float, default=1.5)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.785)
    parser.add_argument('--ns', type=str, default='nautilus')
    parser.add_argument('--threshold', type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    node = DualGoalSender(
        args.x, args.y, args.z, args.roll, args.pitch, args.yaw,
        args.ns, args.threshold,
    )
    rclpy.spin(node)


if __name__ == '__main__':
    main()
