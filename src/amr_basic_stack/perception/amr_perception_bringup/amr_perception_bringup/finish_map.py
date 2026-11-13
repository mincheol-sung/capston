#!/usr/bin/env python3
"""매핑 종료 + 맵 저장 원클릭 노드.

RViz 의 "2D Goal Pose"(/goal_pose)를 누르면 아래를 순서대로 실행한다.

  0. /get_trajectory_states 로 현재 활성 trajectory id 조회
  1. 그 trajectory 를 FinishTrajectory 로 종료
  2. WriteState 로 .pbstream 저장 (기존 맵을 덮지 않도록 자동 버전 증가)
  3. .pbstream 에서 점유격자를 렌더링해 .png + .yaml 생성
  4. 맵을 쓰는 패키지를 재설치해 바로 사용 가능하게 함

저장 위치 기본값은 planner/global_planner/maps/<map>/ 이다. global_planner 는
/map 토픽이 아니라 이 디렉터리의 .yaml/.png 를 직접 읽으므로, 저장하는 즉시
경로 생성(global_line_generator)에 쓸 수 있다.

모든 cartographer 서비스 호출은 rclpy async client + add_done_callback 으로
연결해 단일 rclpy.spin() 루프 안에서만 돈다(재진입 spin 없음).

주의: cartographer_pbstream_to_ros_map 이 만드는 yaml 에는 global_planner 가
      쓰는 start / goal 키가 없다. 저장 후 직접 추가해야 한다(로그로 안내함).
"""

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from cartographer_ros_msgs.srv import (FinishTrajectory, WriteState,
                                       GetTrajectoryStates)
from geometry_msgs.msg import PoseStamped


class FinishMapNode(Node):

    def __init__(self):
        super().__init__('finish_map_node')

        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('maps_dir', '')
        # cartographer_occupancy_grid_node 의 -resolution 과 반드시 같아야 한다.
        self.declare_parameter('resolution', 0.05)
        # 저장 후 재설치할 패키지. 빈 문자열이면 건너뛴다.
        self.declare_parameter('rebuild_package', 'global_planner')

        self.map_name = self.get_parameter('map_name').value
        self.maps_dir = self.get_parameter('maps_dir').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.rebuild_package = self.get_parameter('rebuild_package').value

        if not self.maps_dir:
            self.get_logger().error('maps_dir 파라미터는 필수입니다')
            sys.exit(1)
        os.makedirs(self.maps_dir, exist_ok=True)

        self.get_logger().info(f'맵 이름: {self.map_name}')
        self.get_logger().info(f'저장 위치: {self.maps_dir}')

        self.finish_trajectory_client = self.create_client(
            FinishTrajectory, '/finish_trajectory')
        self.write_state_client = self.create_client(WriteState, '/write_state')
        self.trajectory_states_client = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states')

        self.get_logger().info('cartographer 서비스 대기 중...')
        if not self.finish_trajectory_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/finish_trajectory 서비스 없음')
        if not self.write_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/write_state 서비스 없음')
        if not self.trajectory_states_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '/get_trajectory_states 서비스 없음 (trajectory id 0 으로 진행)')

        # 실제로 디스크에 쓴 basename. 자동 버전 증가로 바뀔 수 있다.
        self.map_basename = self.map_name

        # RViz "2D Goal Pose" 는 /goal_pose 로 발행한다. 네트워크 너머로도
        # 유실되지 않도록 RELIABLE 로 받는다.
        goal_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self.goal_pose_callback, goal_pose_qos)

        self.should_save = False
        self.timer = self.create_timer(0.1, self.check_and_save)

        self.get_logger().warning('\n' + '=' * 62)
        self.get_logger().warning(
            "매핑을 끝내고 맵을 저장하려면 RViz 에서 '2D Goal Pose' 를 클릭하세요")
        self.get_logger().warning('=' * 62 + '\n')

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info('/goal_pose 수신. 맵 저장을 시작합니다...')
        self.should_save = True

    def check_and_save(self):
        if self.should_save:
            self.timer.cancel()
            self.finish_and_save_map()

    def finish_and_save_map(self):
        if self.trajectory_states_client.service_is_ready():
            self.get_logger().info('1/3: 활성 trajectory id 조회...')
            future = self.trajectory_states_client.call_async(
                GetTrajectoryStates.Request())
            future.add_done_callback(self.trajectory_states_callback)
        else:
            self.get_logger().warn('/get_trajectory_states 미준비. id 0 으로 진행')
            self.finish_trajectory(0)

    def trajectory_states_callback(self, future):
        trajectory_id = 0
        try:
            ids = future.result().trajectory_states.trajectory_id
            if len(ids):
                trajectory_id = ids[-1]
        except Exception as exc:
            self.get_logger().error(f'get_trajectory_states 실패({exc}). id 0 으로 진행')
        self.finish_trajectory(trajectory_id)

    def finish_trajectory(self, trajectory_id):
        self.get_logger().info(f'1/3: trajectory {trajectory_id} 종료...')
        req = FinishTrajectory.Request()
        req.trajectory_id = int(trajectory_id)
        future = self.finish_trajectory_client.call_async(req)
        future.add_done_callback(self.finish_trajectory_callback)

    def finish_trajectory_callback(self, future):
        try:
            if future.result() is not None:
                self.get_logger().info('trajectory 종료 완료')
                self.save_pbstream()
            else:
                self.get_logger().error('trajectory 종료 실패')
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            self.get_logger().error(f'trajectory 종료 오류: {exc}')

    def save_pbstream(self):
        """이름이 겹치면 <map>_v2, _v3 ... 로 피해서 저장한다."""
        basename = self.map_name
        pbstream_path = os.path.join(self.maps_dir, f'{basename}.pbstream')
        version = 2
        while os.path.exists(pbstream_path):
            basename = f'{self.map_name}_v{version}'
            pbstream_path = os.path.join(self.maps_dir, f'{basename}.pbstream')
            version += 1
        self.map_basename = basename
        if basename != self.map_name:
            self.get_logger().warn(
                f'{self.map_name}.pbstream 이 이미 있어 {basename}.pbstream 으로 저장합니다')

        self.get_logger().info(f'2/3: pbstream 저장 -> {pbstream_path}')
        req = WriteState.Request()
        req.filename = pbstream_path
        req.include_unfinished_submaps = True
        future = self.write_state_client.call_async(req)
        future.add_done_callback(self.write_state_callback)

    def write_state_callback(self, future):
        try:
            if future.result() is not None:
                self.get_logger().info('pbstream 저장 완료')
                self.save_occupancy_grid()
            else:
                self.get_logger().error('pbstream 저장 실패')
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            self.get_logger().error(f'pbstream 저장 오류: {exc}')

    def save_occupancy_grid(self):
        """저장된 pbstream 에서 점유격자를 렌더링한다.

        라이브 /map 토픽을 스냅샷하지 않고 pbstream 에서 렌더링하는 이유:
        FinishTrajectory 직후 occupancy_grid_node 는 재구축 중이라 부분 격자만
        내보낼 수 있다. pbstream 은 항상 최적화 완료된 전체 맵을 갖고 있다.

        cartographer_pbstream_to_ros_map 은 PGM 만 내보내므로, 이 스택의 다른
        맵들과 같은 PNG 로 변환하고 PGM 은 지운다.
        """
        map_path = os.path.join(self.maps_dir, self.map_basename)
        pbstream_path = f'{map_path}.pbstream'
        pgm_path = f'{map_path}.pgm'
        png_path = f'{map_path}.png'
        yaml_path = f'{map_path}.yaml'

        self.get_logger().info(f'3/3: 점유격자 렌더링 -> {png_path} / {yaml_path}')
        try:
            cmd = ['ros2', 'run', 'cartographer_ros',
                   'cartographer_pbstream_to_ros_map',
                   '-pbstream_filename', pbstream_path,
                   '-map_filestem', map_path,
                   '-resolution', str(self.resolution)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)

            if result.returncode == 0 and os.path.exists(pgm_path):
                self._pgm_to_png(pgm_path, png_path)
                os.remove(pgm_path)
                # 도구가 yaml 의 image 에 절대경로를 넣는다. colcon 이 install/ 로
                # 복사한 뒤에도 맵이 깨지지 않도록 상대 파일명으로 바꾼다.
                self._fix_yaml_image_path(yaml_path, f'{self.map_basename}.png')
                self.get_logger().info(f'맵 이미지 저장: {png_path}')
                self.get_logger().info(f'맵 YAML 저장: {yaml_path}')
            else:
                self.get_logger().error(
                    f'pbstream 렌더링 실패: {result.stderr or result.stdout}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('pbstream_to_ros_map 시간 초과')
        except Exception as exc:
            self.get_logger().error(f'맵 이미지 생성 오류: {exc}')

        self.get_logger().info('\n' + '=' * 62)
        self.get_logger().info('맵 저장 완료')
        self.get_logger().info(f'  - {pbstream_path}   (측위 모드 입력)')
        self.get_logger().info(f'  - {png_path}')
        self.get_logger().info(f'  - {yaml_path}')
        self.get_logger().warning(
            'global_planner 로 경로를 뽑으려면 이 yaml 에 start / goal 키를 '
            '직접 추가해야 합니다. 예)  start: [-4.45, 4.45, 0]')
        self.get_logger().info('=' * 62 + '\n')

        self.rebuild_workspace()

    @staticmethod
    def _pgm_to_png(pgm_path, png_path):
        from PIL import Image
        Image.open(pgm_path).convert('L').save(png_path)

    def _fix_yaml_image_path(self, yaml_path, image_basename):
        try:
            with open(yaml_path, 'r') as f:
                lines = f.readlines()
            with open(yaml_path, 'w') as f:
                for line in lines:
                    if line.startswith('image:'):
                        f.write(f'image: {image_basename}\n')
                    else:
                        f.write(line)
        except Exception as exc:
            self.get_logger().warn(f'yaml image 경로 재작성 실패: {exc}')

    def _find_workspace_dir(self):
        """maps_dir 에서 위로 올라가며 colcon 워크스페이스 루트를 찾는다."""
        path = os.path.abspath(self.maps_dir)
        while path != '/':
            if os.path.isdir(os.path.join(path, 'src')) and \
                    os.path.basename(path).endswith('_ws'):
                return path
            path = os.path.dirname(path)
        return None

    def rebuild_workspace(self):
        """맵을 담은 패키지만 재설치해 새 맵을 바로 쓸 수 있게 한다."""
        if not self.rebuild_package:
            self.get_logger().info('rebuild_package 가 비어 있어 재빌드를 건너뜁니다')
            raise KeyboardInterrupt

        ws_dir = self._find_workspace_dir()
        if ws_dir is None:
            self.get_logger().warn(
                f'{self.maps_dir} 위에서 워크스페이스 루트를 못 찾아 재빌드를 건너뜁니다. '
                f'직접 실행하세요: colcon build --packages-select {self.rebuild_package}')
            raise KeyboardInterrupt

        self.get_logger().info(f'{self.rebuild_package} 재빌드 ({ws_dir})...')
        cmd = ('colcon build --symlink-install --packages-select '
               f'{self.rebuild_package}')
        try:
            result = subprocess.run(['bash', '-c', cmd], cwd=ws_dir, timeout=300)
            if result.returncode == 0:
                self.get_logger().info('재빌드 완료. 새 맵을 바로 쓸 수 있습니다.')
            else:
                self.get_logger().error(f'colcon build 실패 (exit {result.returncode})')
        except subprocess.TimeoutExpired:
            self.get_logger().error('colcon build 시간 초과 (>5분)')
        except Exception as exc:
            self.get_logger().error(f'colcon build 오류: {exc}')

        self.get_logger().info('finish_map 노드를 종료합니다...')
        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = FinishMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
