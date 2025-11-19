"""
Simple helper for sending Nav2 NavigateToPose goals.
Used conceptually in bt_sequence_node.
"""

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MoveToGoalClient:
    def __init__(self, node, action_name: str):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)

    def wait_for_server(self, timeout_sec=10.0):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal_and_wait(self, pose: PoseStamped):
        """
        Send a Nav2 goal and block until result.
        Returns True if succeeded, False otherwise.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        node = self._node
        node.get_logger().info(
            f'[MoveToGoalClient] Sending goal to '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        future = self._client.send_goal_async(goal_msg)
        rclpy = __import__('rclpy')

        rclpy.spin_until_future_complete(node, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            node.get_logger().error('[MoveToGoalClient] Goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        if result.status == 0:
            node.get_logger().info('[MoveToGoalClient] Goal succeeded.')
            return True
        else:
            node.get_logger().error(
                f'[MoveToGoalClient] Goal failed with status {result.status}'
            )
            return False
