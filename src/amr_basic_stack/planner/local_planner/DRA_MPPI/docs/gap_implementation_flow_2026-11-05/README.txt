구현된 gap 전략: 실제 LiDAR 단계별 PNG (2026-11-05)

mpc_chain_20260923_135606  +212.271s  |  b2_100
실제 기록의 시간 보정 scan·지도·nominal 재사용 | 동적 예측 미주입 | 실주행·전체 5Hz 검증 아님

01_actual_lidar.png
02_reachable_space.png
03_disparity_gaps.png
04_midpoint_seeds.png
05_candidate_decisions.png
06_budget_and_rollouts.png
07_final_selection.png
08_output_and_next_nominal.png
00_overall_flow.png

같은 장면과 같은 좌표계를 모든 단계에 사용. gap별 색은 고정.
그림의 seed·선택 사유·500개 rollout·최종 대표는 현재 production 구현의 실제 출력.
scan은 저장된 deskew 좌표, upstream map은 저장된 융합 grid를 재사용.
온라인 ROS freshness/TF 검사는 재실행하지 않음. 동적 예측은 주입하지 않음.
다음 nominal은 제어열 shift를 표시하며, 다음 센서 장면/폐루프 실주행 결과가 아님.
그림 생성은 ROS 토픽/로봇 명령을 발행하지 않음.

재현: ROS와 workspace setup을 source한 뒤 패키지 디렉터리에서
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 tools/render_production_gap_flow.py
