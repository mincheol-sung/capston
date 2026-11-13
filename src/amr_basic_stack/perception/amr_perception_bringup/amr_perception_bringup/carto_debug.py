#!/usr/bin/env python3
"""cartographer 상태 진단 뷰어 (amr_basic).

두 가지를 한 화면에 보여준다.

1) 센서 입력 상태
   cartographer 가 조용히 아무것도 안 하는 원인은 대부분 여기다. /scan, /odom,
   /camera/imu 가 실제로 들어오는지, TF 가 이어져 있는지 먼저 본다.
   특히 IMU 는 frame_id 인 camera_imu_optical_frame 이 TF 에 없으면 메시지가
   전량 폐기되면서도 에러가 안 뜨는 경우가 있다.

2) /read_metrics 기반 매칭 건강도
   얼어붙은 맵과의 정합이 잘 되는지 보는 숫자만 뽑는다.
     * 포즈그래프 구속 개수(다른 궤적 = 얼어붙은 맵 / 같은 궤적 = 루프클로저)
     * local / global 탐색 시도 대비 성공
     * 채택된 구속의 점수 분포(min_score 여유)
     * 서브맵 상태, 백엔드 작업 큐 압력

사용법 (빌드 후):
    ros2 run amr_perception_bringup carto_debug
    ros2 run amr_perception_bringup carto_debug --watch 2

cartographer_node 가 -collect_metrics 로 떠 있어야 2)가 나온다.
launch 파일은 기본으로 붙여준다.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from cartographer_ros_msgs.srv import ReadMetrics
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


# ─────────────────────────── /read_metrics 파싱 ───────────────────────────

def to_dict(labels):
    return {l.key: l.value for l in labels}


def find(families, name):
    for f in families:
        if f.name == name:
            return f
    return None


def metric_value(family, want):
    """labels 가 want 를 모두 포함하는 metric 의 값."""
    if family is None:
        return None
    for m in family.metrics:
        lab = to_dict(m.labels)
        if all(lab.get(k) == v for k, v in want.items()):
            return m.value
    return None


def histogram_line(family, want, lo_edge=0.0):
    """히스토그램에서 비어있지 않은 버킷만 'lo-hi:count' 로 렌더링."""
    if family is None:
        return 'n/a'
    for m in family.metrics:
        lab = to_dict(m.labels)
        if all(lab.get(k) == v for k, v in want.items()):
            parts, prev, total = [], lo_edge, 0
            for b in m.counts_by_bucket:
                if b.count > 0:
                    hi = 'inf' if b.bucket_boundary == float('inf') \
                        else f'{b.bucket_boundary:.2f}'
                    parts.append(f'{prev:.2f}-{hi}:{int(b.count)}')
                    total += int(b.count)
                prev = b.bucket_boundary
            return f'total {total} | ' + ('  '.join(parts) if parts else '-')
    return 'n/a'


def metrics_snapshot(node, client):
    if client is None or not client.service_is_ready():
        return ('  /read_metrics 없음 — cartographer_node 가 -collect_metrics '
                '없이 떴거나 아직 안 떴습니다')

    future = client.call_async(ReadMetrics.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    res = future.result()
    if res is None:
        return '  ERROR: /read_metrics 호출 시간 초과'
    if res.status.code != 0:
        return f'  ERROR: {res.status.message}'
    fam = res.metric_families

    pg_con = find(fam, 'mapping_2d_pose_graph_constraints')
    cb_con = find(fam, 'mapping_constraints_constraint_builder_2d_constraints')
    cb_scores = find(fam, 'mapping_constraints_constraint_builder_2d_scores')
    cb_queue = find(fam, 'mapping_constraints_constraint_builder_2d_queue_length')
    submaps = find(fam, 'mapping_2d_pose_graph_submaps')
    wq_size = find(fam, 'mapping_2d_pose_graph_work_queue_size')
    wq_delay = find(fam, 'mapping_2d_pose_graph_work_queue_delay')
    rt_ratio = find(fam, 'mapping_2d_local_trajectory_builder_real_time_ratio')
    latency = find(fam, 'mapping_2d_local_trajectory_builder_latency')

    diff = metric_value(pg_con, {'trajectory': 'different'})
    same = metric_value(pg_con, {'trajectory': 'same'})
    l_s = metric_value(cb_con, {'search_region': 'local', 'matcher': 'searched'}) or 0
    l_f = metric_value(cb_con, {'search_region': 'local', 'matcher': 'found'}) or 0
    g_s = metric_value(cb_con, {'search_region': 'global', 'matcher': 'searched'}) or 0
    g_f = metric_value(cb_con, {'search_region': 'global', 'matcher': 'found'}) or 0
    l_pct = f'{100.0 * l_f / l_s:.1f}%' if l_s else '-'

    rt = metric_value(rt_ratio, {})
    rt_warn = '   <-- 1.0 미만이면 실시간 처리 실패' if (rt is not None and rt < 1.0) else ''

    return '\n'.join([
        f"  그래프 구속       : 얼어붙은맵(다른궤적) {int(diff) if diff is not None else '?'}"
        f"   루프클로저(같은궤적) {int(same) if same is not None else '?'}",
        f'  local 탐색        : 성공 {int(l_f)} / 시도 {int(l_s)}  ({l_pct})',
        f'  global 탐색       : 성공 {int(g_f)} / 시도 {int(g_s)}'
        f"{'' if g_f else '   <-- 부팅 직후 여기서 성공이 나와야 맵에 붙는다'}",
        f"  채택 점수 분포    : {histogram_line(cb_scores, {'search_region': 'local'})}",
        f"  서브맵            : active {metric_value(submaps, {'state': 'active'})}, "
        f"frozen {metric_value(submaps, {'state': 'frozen'})}",
        f'  매처 큐           : {metric_value(cb_queue, {})} 대기',
        f"  백엔드 큐         : size {metric_value(wq_size, {})}, "
        f"delay {metric_value(wq_delay, {})} s",
        f'  프론트엔드 rt비율 : {rt}   (배치 처리시간 {metric_value(latency, {})} s){rt_warn}',
    ])


# ─────────────────────────── 센서 / TF 점검 ───────────────────────────

TOPICS = [
    ('/scan', LaserScan, '병합 라이다 (dual_scan_merger)'),
    ('/odom', Odometry, '휠 오도메트리 (roboclaw)'),
    ('/camera/imu', Imu, 'D455 IMU'),
    ('/imu/base_link', Imu, 'body 프레임 회전본 (EKF 입력)'),
    ('/tracked_pose', PoseStamped, 'cartographer 출력'),
]

TF_CHECKS = [
    ('map', 'odom', 'ekf_carto 가 발행'),
    ('odom', 'base_link', 'ekf_filter_node(기존 EKF) 가 발행'),
    ('base_link', 'base_scan', 'URDF'),
    ('base_link', 'camera_imu_optical_frame', 'URDF — 없으면 IMU 전량 폐기'),
]


class SensorProbe(Node):
    """짧게 구독해 수신율을 재는 노드."""

    def __init__(self):
        super().__init__('carto_debug')
        self.counts = {name: 0 for name, _, _ in TOPICS}
        # 센서 토픽은 BEST_EFFORT 로 오는 경우가 있어 호환 넓은 쪽으로 구독한다.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        for name, msg_type, _ in TOPICS:
            self.create_subscription(
                msg_type, name, self._make_cb(name), qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _make_cb(self, name):
        def cb(_msg):
            self.counts[name] += 1
        return cb

    def measure(self, window_sec=1.0):
        for key in self.counts:
            self.counts[key] = 0
        end = time.time() + window_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return {k: v / window_sec for k, v in self.counts.items()}

    def sensor_report(self, window_sec=1.0):
        rates = self.measure(window_sec)
        lines = []
        for name, _, desc in TOPICS:
            hz = rates[name]
            pubs = self.count_publishers(name)
            if pubs == 0:
                status = 'X  발행자 없음'
            elif hz == 0.0:
                status = 'X  발행자는 있으나 수신 0'
            else:
                status = f'{hz:6.1f} Hz'
            lines.append(f'  {name:<16} {status:<24} {desc}')

        lines.append('')
        for parent, child, owner in TF_CHECKS:
            try:
                self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                mark = 'OK'
            except tf2_ros.TransformException:
                mark = 'X '
            lines.append(f'  {mark}  {parent:<10} -> {child:<26} {owner}')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--watch', type=float, metavar='SEC', default=None,
                        help='SEC 초마다 갱신 (기본: 1회 출력)')
    parser.add_argument('--window', type=float, default=1.0,
                        help='토픽 수신율 측정 창 길이(초). 기본 1.0')
    args = parser.parse_args()

    rclpy.init()
    node = SensorProbe()

    client = node.create_client(ReadMetrics, '/read_metrics')
    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().warn(
            '/read_metrics 없음 (cartographer_node 가 -collect_metrics 로 떠 있나요?)')
        client = None

    try:
        while True:
            print(time.strftime('[%H:%M:%S] ') + '=' * 50)
            print(' 센서 입력 / TF')
            print(node.sensor_report(args.window))
            print(' cartographer 매칭 건강도')
            print(metrics_snapshot(node, client))
            if args.watch is None:
                break
            print()
            time.sleep(args.watch)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
