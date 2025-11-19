"""
Helper conceptually used for returning robot to a saved start pose.
In practice, bt_sequence_node calls Nav2 directly with the saved pose.
"""

from geometry_msgs.msg import PoseStamped
from .move_to_goal import MoveToGoalClient


class ReturnToStartHelper:
    def __init__(self, node, action_name: str):
        self._node = node
        self._client = MoveToGoalClient(node, action_name)

    def return_to(self, start_pose: PoseStamped) -> bool:
        if start_pose is None:
            self._node.get_logger().warn('[ReturnToStartHelper] No start pose.')
            return False

        if not self._client.wait_for_server():
            self._node.get_logger().error(
                '[ReturnToStartHelper] Nav2 server not available.'
            )
            return False

        return self._client.send_goal_and_wait(start_pose)
