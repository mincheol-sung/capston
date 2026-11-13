같은 장면 방향별 거리 비교
01_fixed_heading.png: 차체 방향 유지(omega=0).
02_recorded_nominal_yaw.png: 저장 nominal 회전 명령을 고정하고 병진 속도를 최적화.
두 그림 모두 조건부 직선 최대 거리이며, 자유 회전/곡선 경로를 포함한 전역 최대가 아니다.
원본 LiDAR+지도 공간 깊이를 재계산하여 기존 결과와 일치함을 확인했다.
회전열과 몸체 좌표 축 제한이 함께 작용하므로 조건 B가 모든 방향에서 A보다 작을 필요는 없다.
d=2m는 상한까지 관측 공간이 확인됨을 의미한다. raw range나 벽 위치가 아니다.
현재 명령은 기록값을 사용. 정지에 가깝지만 정확한 0으로 바꾸지 않았다.
표는 원본 G1~G6 중앙 방향. CSV는 360방향 전부. NPZ는 구동 최대 및 공간 제한 후 제어열/궤적.
기존 seed 여유 10cm, 도달거리 90%, desired_speed 0.75는 최대 구동 거리 계산에 적용하지 않았다.
종단 정지는 요구하지 않는다. 해당 시간까지 도달하는 최대 거리이며 정지 가능 거리 계산은 아니다.
이 계산은 5Hz 성능 검증도 새 비용 최적화 구현도 아니다. 운영 코드/YAML 미변경.
재생성: PYTHONPATH=planner/local_planner/DRA_MPPI /usr/bin/python3 planner/local_planner/DRA_MPPI/tools/render_gap_directional_reach.py
