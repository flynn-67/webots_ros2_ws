import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from turtlebot3_camera_service_interfaces.srv import ImageCapture

import math
from datetime import datetime


class BTSequenceNode(Node):
    """
    /bt/goal_pose (PoseStamped)를 입력으로 받아서 순서대로 수행:
      1) 현재 위치를 시작점으로 저장 (amcl_pose 기준)
      2) Nav2 NavigateToPose 액션으로 goal까지 이동
      3) /image_capture 서비스 호출해서 이미지 저장
      4) Nav2 NavigateToPose 액션으로 시작점으로 복귀
    """

    def __init__(self):
        super().__init__('bt_sequence_node')

        # 파라미터: Nav2 네임스페이스 (예: "/TurtleBot3Burger")
        self.declare_parameter('nav2_namespace', '/TurtleBot3Burger')
        ns = self.get_parameter('nav2_namespace').get_parameter_value().string_value
        self.nav_ns = ns if ns != '' else ''

        # Nav2 액션 클라이언트
        action_name = f'{self.nav_ns}/navigate_to_pose' if self.nav_ns else '/navigate_to_pose'
        self._nav_client = ActionClient(self, NavigateToPose, action_name)

        # 이미지 캡쳐 서비스 클라이언트
        self._image_cli = self.create_client(ImageCapture, 'image_capture')

        # /bt/goal_pose 구독
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/bt/goal_pose',
            self._goal_callback,
            10
        )

        # /amcl_pose 구독해서 현재 위치 계속 저장
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.nav_ns}/amcl_pose' if self.nav_ns else '/amcl_pose',
            self._amcl_callback,
            10
        )

        self._latest_amcl_pose = None
        self._start_pose = None
        self._busy = False

        self.get_logger().info(
            f'BT Sequence Node started. Waiting for /bt/goal_pose ... '
            f'(Nav2 action: {action_name})'
        )

    # ---------- 콜백들 ----------

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self._latest_amcl_pose = msg

    def _goal_callback(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn('Already running a BT sequence, ignoring new goal.')
            return

        self._busy = True
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.get_logger().info(
            f'BT triggered by /bt/goal_pose: ({goal_x:.2f}, {goal_y:.2f})'
        )

        # 시작점 저장
        if self._latest_amcl_pose is not None:
            self._start_pose = self._pose_from_amcl(self._latest_amcl_pose)
            self.get_logger().info(
                f'Start pose saved at '
                f'({self._start_pose.pose.position.x:.2f}, '
                f'{self._start_pose.pose.position.y:.2f})'
            )
        else:
            # amcl_pose를 아직 못 받았으면 (0,0)을 시작점으로 가정
            self.get_logger().warn(
                'No amcl_pose received yet. Assuming start pose at (0,0).'
            )
            self._start_pose = PoseStamped()
            self._start_pose.header.frame_id = 'map'
            self._start_pose.pose.orientation.w = 1.0

        # 비동기 시퀀스 실행
        self.create_task(self._run_sequence(msg))

    # ---------- 유틸 ----------

    def _pose_from_amcl(self, amcl: PoseWithCovarianceStamped) -> PoseStamped:
        ps = PoseStamped()
        ps.header = amcl.header
        ps.pose = amcl.pose.pose
        return ps

    def create_task(self, coro):
        """
        rclpy에는 asyncio 루프가 없어서, 타이머를 이용해 코루틴을 한 step씩 실행.
        """
        def step(fut=[coro]):
            try:
                fut[0].__next__()
            except StopIteration:
                timer.cancel()

        timer = self.create_timer(0.01, step)

    # ---------- 시퀀스 핵심 로직 ----------

    def _wait_for_nav2(self):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return False
        return True

    def _wait_for_service(self):
        if not self._image_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/image_capture service not available!')
            return False
        return True

    def _send_nav_goal(self, target_pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.get_logger().info(
            f'Sending Nav2 goal to ({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f})'
        )

        send_future = self._nav_client.send_goal_async(goal_msg)
        done = False

        def goal_response_cb(fut):
            nonlocal done
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('Nav2 goal rejected!')
                done = True
                return
            self.get_logger().info('Nav2 goal accepted.')
            result_future = goal_handle.get_result_async()

            def res_cb(rf):
                nonlocal done
                status = rf.result().status
                if status == 0:
                    self.get_logger().info('Nav2 goal succeeded.')
                else:
                    self.get_logger().error(f'Nav2 goal failed with status {status}')
                done = True

            result_future.add_done_callback(res_cb)

        send_future.add_done_callback(goal_response_cb)

        # yield로 완료까지 기다리기
        while not done:
            yield

    def _call_image_capture(self, filename: str):
        if not self._wait_for_service():
            return

        req = ImageCapture.Request()
        req.filename = filename
        self.get_logger().info(f'Calling /image_capture with filename="{filename}"')
        future = self._image_cli.call_async(req)
        done = False

        def cb(fut):
            nonlocal done
            resp = fut.result()
            if resp.success:
                self.get_logger().info(f'ImageCapture success: {resp.message}')
            else:
                self.get_logger().error(f'ImageCapture failed: {resp.message}')
            done = True

        future.add_done_callback(cb)

        while not done:
            yield

    # ---------- 전체 시퀀스 ----------

    def _run_sequence(self, goal_pose: PoseStamped):
        try:
            if not self._wait_for_nav2():
                self._busy = False
                return

            # 1) goal로 이동
            yield from self._send_nav_goal(goal_pose)

            # 2) 이미지 캡쳐
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'bt_capture_{timestamp}.png'
            yield from self._call_image_capture(filename)

            # 3) 시작점으로 복귀
            if self._start_pose is not None:
                self.get_logger().info('Returning to start pose...')
                yield from self._send_nav_goal(self._start_pose)
            else:
                self.get_logger().warn('No start pose stored, skipping return_to_start.')

            self.get_logger().info('BT sequence finished.')

        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = BTSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

