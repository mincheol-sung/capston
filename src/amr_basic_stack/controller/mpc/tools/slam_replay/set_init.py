#!/usr/bin/python3
"""Cartographer 의 지금 궤적을 끝내고 준 위치에서 새로 시작 (set_initial_pose_node 와 같은 서비스 순서, EKF 리셋은 없음).
사용: set_init.py 설정폴더 lua x y qz qw   — 위치는 /tracked_pose 와 같은 추적점 기준, 0 번(지도) 궤적 좌표."""
import sys
import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from rclpy.node import Node


def main():
    cfg, lua = sys.argv[1], sys.argv[2]
    x, y, qz, qw = map(float, sys.argv[3:7])
    rclpy.init()
    n = Node("set_init_replay")

    def call(srv, name, req):
        cli = n.create_client(srv, name)
        if not cli.wait_for_service(timeout_sec=30.0):
            sys.exit(f"{name} 서비스 없음 — cartographer_node 가 떴는지 확인")
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
        return f.result()

    st = call(GetTrajectoryStates, "/get_trajectory_states", GetTrajectoryStates.Request())
    for tid, s in zip(st.trajectory_states.trajectory_id, st.trajectory_states.trajectory_state):
        if s == 0:   # ACTIVE
            call(FinishTrajectory, "/finish_trajectory", FinishTrajectory.Request(trajectory_id=tid))
    r = StartTrajectory.Request()
    r.configuration_directory, r.configuration_basename, r.use_initial_pose = cfg, lua, True
    r.initial_pose.position.x, r.initial_pose.position.y = x, y
    r.initial_pose.orientation.z, r.initial_pose.orientation.w = qz, qw
    r.relative_to_trajectory_id = 0
    res = call(StartTrajectory, "/start_trajectory", r)
    print(f"start_trajectory {res.status.code} {res.status.message} id {res.trajectory_id}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
