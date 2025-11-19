"""
Subscriber for /amcl_pose that keeps track of the latest robot pose
to be used as a "start pose" in homework.
"""

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class StartPoseSaver:
    def __init__(self, node, topic='/amcl_pose'):
        self._node = node
        self._latest = None
        self._sub = node.create_subscription(
            PoseWithCovarianceStamped,
            topic,
            self._callback,
            10
        )

    def _callback(self, msg: PoseWithCovarianceStamped):
        self._latest = msg

    def get_start_pose(self) -> PoseStamped | None:
        if self._latest is None:
            return None
        ps = PoseStamped()
        ps.header = self._latest.header
        ps.pose = self._latest.pose.pose
        return ps
