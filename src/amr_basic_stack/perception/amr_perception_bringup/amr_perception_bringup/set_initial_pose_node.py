#!/usr/bin/env python3
"""측위 모드의 초기 위치 설정 노드.

cartographer 는 AMCL 처럼 /initialpose 를 그냥 먹지 않는다. 초기 위치를 주려면
현재 trajectory 를 끝내고(FinishTrajectory) 초기 pose 를 지정해 새 trajectory 를
시작해야(StartTrajectory with use_initial_pose) 한다. 이 노드가 RViz 의
"2D Pose Estimate" 클릭을 받아 그 두 서비스를 순서대로 호출한다.

relative_to_trajectory_id = 0 인 이유: 0 번은 매핑 때 저장돼 pbstream 으로
불러온 얼어붙은 궤적이다. 새 궤적의 초기 pose 를 그 맵 좌표계 기준으로 준다.

RViz QoS 주의: "2D Pose Estimate" 는 /initialpose 를 BEST_EFFORT 로 발행한다.
기본값(depth=10)으로 만든 RELIABLE 구독자는 QoS 비호환이라 아무것도 못 받고,
버튼을 눌러도 콜백이 안 뜬다. BEST_EFFORT 구독자는 양쪽 다 받으므로 그걸 쓴다.

★ EKF 리셋이 반드시 따라와야 하는 이유
cartographer 가 새 궤적으로 제자리를 찾아도, map -> odom TF 를 실제로 내는 것은
ekf_carto(robot_localization)다. 그 EKF 에는 마할라노비스 게이팅
(ekf_carto_localization.yaml 의 odom1_pose_rejection_threshold)이 걸려 있어,
    innov^T (H P H^T + R)^-1 innov >= threshold^2  ->  측정 폐기
로 판정한다. cov 0.01 [m^2], threshold 3.0 이면 허용 점프가 약 0.3~0.42 m,
yaw 17 도뿐이다. 재측위는 보통 그보다 훨씬 크게 옮기므로 카트로그래퍼의 정답이
매번 이상치로 걸러지고, 차이가 남아 있는 한 영원히 다시 안 붙는다.
그래서 StartTrajectory 성공 직후 같은 pose 를 ekf_carto 의 set_pose 로 보내
필터 상태와 공분산을 통째로 재초기화한다. 게이트 자체를 푸는 것은 주행 중
오정합 순간이동까지 받아들이게 되므로 하지 않는다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.msg import StatusCode
from cartographer_ros_msgs.srv import (FinishTrajectory, StartTrajectory,
                                       GetTrajectoryStates)


class SetInitialPose(Node):

    def __init__(self):
        super().__init__('set_initial_pose_node')

        self.declare_parameter('config_dir', '')
        self.declare_parameter('config_base', 'localization_2d.lua')
        # ekf_carto 의 set_pose 는 launch 에서 이 이름으로 리맵돼 있다. 리맵을
        # 안 하면 양쪽 EKF 다 루트 네임스페이스라 /set_pose 가 되어, odom 레벨
        # ekf_filter_node 까지 map 좌표 pose 로 리셋돼 TF 트리가 망가진다.
        self.declare_parameter('ekf_set_pose_topic', '/ekf_carto/set_pose')
        self.declare_parameter('map_frame', 'map')

        self.config_dir = self.get_parameter('config_dir').value
        self.config_base = self.get_parameter('config_base').value
        self.map_frame = self.get_parameter('map_frame').value
        ekf_set_pose_topic = self.get_parameter('ekf_set_pose_topic').value

        self.get_logger().info(f'설정 디렉터리: {self.config_dir}')
        self.get_logger().info(f'설정 파일: {self.config_base}')

        self.finish_trajectory_client = self.create_client(
            FinishTrajectory, '/finish_trajectory')
        self.start_trajectory_client = self.create_client(
            StartTrajectory, '/start_trajectory')
        self.trajectory_states_client = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states')

        self.get_logger().info('cartographer 서비스 대기 중...')
        self.finish_trajectory_client.wait_for_service(timeout_sec=10.0)
        self.start_trajectory_client.wait_for_service(timeout_sec=10.0)
        self.trajectory_states_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('cartographer 서비스 준비됨')

        initialpose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initial_pose_callback, initialpose_qos)

        # robot_localization 의 set_pose 구독자는 RELIABLE(KeepLast 1) 이므로
        # 여기는 기본 QoS 로 둔다. /initialpose 의 BEST_EFFORT 와 다르다.
        self.set_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, ekf_set_pose_topic, 10)
        self.get_logger().info(f'EKF 리셋 토픽: {ekf_set_pose_topic}')

        self.current_pose_msg = None
        self.get_logger().warning('\n' + '=' * 62)
        self.get_logger().warning(
            "RViz 에서 '2D Pose Estimate' 로 로봇의 현재 위치를 찍으세요")
        self.get_logger().warning('=' * 62 + '\n')

    def initial_pose_callback(self, msg):
        # RViz 는 클릭 시점의 Fixed Frame 으로 스탬프를 찍는다. 이 노드는 좌표를
        # 맵 좌표계(relative_to_trajectory_id = 0) 기준으로 그대로 넣으므로,
        # Fixed Frame 이 odom 이면 위치가 통째로 틀어진다. TF 로 고쳐 넣지 않는
        # 이유는 지금 틀어져 있는 게 바로 그 map -> odom TF 이기 때문이다.
        frame = msg.header.frame_id or self.map_frame
        if frame != self.map_frame:
            self.get_logger().error(
                f"\n{'=' * 62}\n"
                f"'2D Pose Estimate' 가 '{frame}' 프레임으로 들어왔습니다. "
                f"이 클릭은 무시합니다.\n"
                f"RViz 왼쪽 위 Global Options > Fixed Frame 을 "
                f"'{self.map_frame}' 으로 바꾸고 다시 찍으세요.\n"
                f"{'=' * 62}")
            return

        p = msg.pose.pose.position
        self.get_logger().info(f'초기 위치 수신: ({p.x:.3f}, {p.y:.3f})')
        self.current_pose_msg = msg

        # 하드코딩된 id 대신 현재 활성 trajectory 를 조회해서 끝낸다.
        # (측위는 보통 1번부터지만, 초기 pose 를 여러 번 다시 찍으면 계속 늘어난다.)
        future = self.trajectory_states_client.call_async(
            GetTrajectoryStates.Request())
        future.add_done_callback(self.trajectory_states_callback)

    def trajectory_states_callback(self, future):
        trajectory_id = 1
        try:
            states = future.result().trajectory_states
            ids = list(states.trajectory_id)
            if ids:
                trajectory_id = ids[-1]
        except Exception as exc:
            self.get_logger().warn(f'get_trajectory_states 실패({exc}). id 1 로 진행')

        self.get_logger().info(f'trajectory {trajectory_id} 종료...')
        req = FinishTrajectory.Request()
        req.trajectory_id = int(trajectory_id)
        finish_future = self.finish_trajectory_client.call_async(req)
        finish_future.add_done_callback(self.finish_trajectory_callback)

    def finish_trajectory_callback(self, future):
        try:
            if future.result() is not None:
                self.get_logger().info('trajectory 종료 완료')
                # 백엔드가 정리될 짧은 여유를 준 뒤 새 궤적을 시작한다.
                self._start_timer = self.create_timer(0.5, self._start_trajectory_once)
            else:
                self.get_logger().warn('trajectory 종료 실패')
        except Exception as exc:
            self.get_logger().error(f'trajectory 종료 오류: {exc}')

    def _start_trajectory_once(self):
        # 0.5 s 주기 타이머라 첫 콜백이 타이머를 파괴하는 사이 두 번째 콜백이 이미
        # 큐에 들어와 있을 수 있다. 그러면 파괴된 핸들에 cancel() 을 불러
        # rclpy InvalidHandle 로 노드가 죽는다 (2026-09-14 Isaac 측위 실측).
        # 한 번만 실행하고, 파괴 중 예외는 무시한다.
        timer = getattr(self, '_start_timer', None)
        if timer is None:
            return
        self._start_timer = None
        try:
            timer.cancel()
            self.destroy_timer(timer)
        except Exception as exc:  # InvalidHandle 등
            self.get_logger().debug(f'start timer 정리 중 무시된 예외: {exc}')
        self.start_new_trajectory()

    def start_new_trajectory(self):
        self.get_logger().info('초기 pose 로 새 trajectory 시작...')
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        req.initial_pose = self.current_pose_msg.pose.pose
        req.relative_to_trajectory_id = 0   # pbstream 으로 불러온 얼어붙은 궤적

        future = self.start_trajectory_client.call_async(req)
        future.add_done_callback(self.start_trajectory_callback)

    def start_trajectory_callback(self, future):
        try:
            result = future.result()
            if result is None:
                self.get_logger().error('새 trajectory 시작 실패(응답 없음)')
                return
            # 응답이 왔다고 성공은 아니다. 실패한 채로 EKF 를 리셋하면
            # cartographer 는 옛 위치에 있는데 TF 만 새 위치로 튄다.
            if result.status.code != StatusCode.OK:
                self.get_logger().error(
                    f'새 trajectory 시작 거부됨(code {result.status.code}): '
                    f'{result.status.message}. EKF 는 건드리지 않습니다.')
                return
            self.get_logger().info(f'새 trajectory 시작됨. ID: {result.trajectory_id}')
            self.reset_ekf_pose()
        except Exception as exc:
            self.get_logger().error(f'trajectory 시작 오류: {exc}')

    def reset_ekf_pose(self):
        """ekf_carto 의 상태를 방금 찍은 pose 로 재초기화한다(docstring 참조)."""
        msg = PoseWithCovarianceStamped()
        # 클릭 시각이 아니라 지금 시각이어야 한다. robot_localization 은
        # set_pose 이후 그보다 오래된 측정을 전부 버리는데, 궤적 재시작 동안
        # 옛 위치에서 나온 /tracked_pose 가 늦게 도착하기 때문이다.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose = self.current_pose_msg.pose.pose

        # 공분산은 EKF 의 초기 P 가 된다. 넉넉히 줘야 리셋 직후 게이트가 열려
        # 첫 cartographer pose 가 들어온다. RViz 기본값이 이미 그 수준이다.
        covariance = list(self.current_pose_msg.pose.covariance)
        if not any(covariance):
            covariance[0] = 0.25     # x   [m^2]
            covariance[7] = 0.25     # y   [m^2]
            covariance[35] = 0.068   # yaw [rad^2]
        msg.pose.covariance = covariance

        self.set_pose_publisher.publish(msg)
        self.get_logger().info(
            'ekf_carto 를 초기 pose 로 리셋했습니다 (map -> odom TF 갱신).')


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('set_initial_pose_node 종료')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
