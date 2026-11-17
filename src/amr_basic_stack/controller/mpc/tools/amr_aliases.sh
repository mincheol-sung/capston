# 실차 토크 실험 짧은 명령 (2026-11-16). ~/.bashrc 에서 source 한다:
#     source /home/smoc/capston/capston/src/amr_basic_stack/controller/mpc/tools/amr_aliases.sh
# 터미널마다 따로 'amr' + export 를 칠 필요가 없다 — 각 명령이 환경(ROS_DOMAIN_ID 81 등)을 스스로 맞춘다.
# 실행 순서·안전 절차는 amr/notes_amr/real_robot_checklist.md 그대로다 (두 사람 정지 절차, 모터 전원).
#
#   명령                     하는 일                                        원래 명령 (real_robot_checklist 의 터미널)
#   amrcan                   CAN 준비 (재부팅 뒤 꼭)                         T1  tools/setup_can.sh
#   amrcanfix                CAN bus-off 복구 (모터 전원을 다시 꽂은 뒤)     can0 down → slcand 종료 → setup_can.sh
#   amrsens                  센서: 라이다 2 개 + 카메라 IMU                  A   sensors.launch.py
#   amrloc                   측위: smoc_lab 지도, 토크용 lua                 B   cartographer.launch.py mode:=localization
#   amrbase                  CAN 노드 + 추정기 (MPC 없음, 기본 동작 시험용)  C   torque_experiment.launch.py use_mpc:=false
#   amrmpc  [인자...]        CAN 노드 + 추정기 + MPC ('a' 무장, 's' 정지)   C   torque_experiment.launch.py
#   amr8    [인자...]        amrmpc + 8자를 무장한 자리(로봇) 기준으로       C   ... origin_mode:=robot (속도 등은 torque_track.yaml)
#   amrrec  [이름]           bag 녹화, 모든 토픽 (영상·점군·카토 표시용 제외) + run_info(CAN·CPU·온도·로그) bags/<날짜>/<시분초>[_이름]  D   tools/record_all.sh
#   amrcheck                 측위 맞추기 ("맞음" 이 나와야 한다)             T1  check_localization.py --apply
#   amrmove [인자...]        기본 동작 시험 (amrbase 일 때만)                T1  basic_motion_test.py
#   amrmu   [인자...]        바닥 μ̂ 토크 램프 시험 (MPC 없을 때만)          T1  mu_ramp_test.py
#   amrviz  [인자...]        RViz (랩 PC 에서)                               torque_viz.launch.py
#   amrall  [인자...]        위 센서·측위·CAN(+MPC)·표시·측위 맞추기를 터미널 하나로 (bag:=true 면 녹화도, 폴더 이름 자동)   real_all.launch.py
#   amrramp [인자...]        바닥 토크 램프 (amrall 스택 + MPC 대신 램프, dir:=fwd|left mu_hat:= bag:=true)  torque_ramp.launch.py
#   amrhelp                  이 표
#   예) amrmpc shape:=line seg_distance:=0.4 seg_dir_deg:=45 speed:=0.10 limit_mode:=friction mu_hat:=0.55
#       amrmove --dirs fwd,back --amps 0.1 --time 0.5
#       amrramp dir:=left mu_hat:=0.6 bag:=ramp_left
#       amrmu --dirs fwd,back,left,right,fl --repeat 3 --csv ~/capston/capston/analysis/torque_mpc_real/2026-11-17_mu/ramp1
#
# 컴퓨터마다 다른 값 (source 전에 export 하면 바뀐다)
#   AMR_WS       작업공간          NUC 기본 /home/smoc/capston/capston
#   AMR_INSTALL  설치 폴더 이름    NUC 기본 install, 랩 PC 는 install_pc (stack_changelog: PC 는 build_pc/install_pc 로 따로 빌드)
#   AMR_PEERS    상대 컴퓨터 IP    비우면 NUC 혼자 (지금까지 실험처럼 ROS_STATIC_PEERS 를 지운다).
#                                  랩 PC 의 RViz 로 볼 때: NUC 에는 랩 PC IP, 랩 PC 에는 NUC IP (192.168.0.132).
#                                  ※ 꺼진 컴퓨터 IP 를 넣어 두면 노드 생성이 느려진 적이 있다 (~/.bashrc 주석: 18 s) — 쓸 때만 넣는다.
# 모든 명령은 서브셸에서 돈다 — 지금 셸의 위치·환경 변수는 바뀌지 않는다.

: "${AMR_WS:=/home/smoc/capston/capston}"
: "${AMR_INSTALL:=install}"
: "${AMR_DOMAIN:=81}"
AMR_MPC_DIR="$AMR_WS/src/amr_basic_stack/controller/mpc"

_amr_env() {
  cd "$AMR_WS" || return 1
  source /opt/ros/jazzy/setup.bash || return 1
  source "$AMR_WS/$AMR_INSTALL/setup.bash" || return 1
  export ROS_DOMAIN_ID="$AMR_DOMAIN"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  # NUC ~/.bashrc 와 같은 CycloneDDS 설정 (참가자 번호 auto — 상대 컴퓨터가 정해진 포트로 찾을 수 있게). 이미 있으면 그대로
  : "${CYCLONEDDS_URI:=<CycloneDDS><Domain id=\"any\"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
  export CYCLONEDDS_URI
  if [ -n "$AMR_PEERS" ]; then
    export ROS_STATIC_PEERS="$AMR_PEERS"
  else
    unset ROS_STATIC_PEERS
  fi
  echo "[amr] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  peers=${ROS_STATIC_PEERS:-(없음, 이 컴퓨터만)}"
}

amrcan()    ( cd "$AMR_MPC_DIR" && sudo -v && bash tools/setup_can.sh )
amrcanfix() ( cd "$AMR_MPC_DIR" || exit 1; sudo -v || exit 1; sudo ip link set can0 down; sudo pkill slcand; bash tools/setup_can.sh )
amrsens()   ( _amr_env && exec ros2 launch amr_bringup sensors.launch.py "$@" )
amrloc()    ( _amr_env && exec ros2 launch amr_perception_bringup cartographer.launch.py mode:=localization map:=smoc_lab lua_basename:=localization_torque_2d.lua "$@" )
amrbase()   ( _amr_env && exec ros2 launch amr_mpc torque_experiment.launch.py plant:=real use_rviz:=false use_mpc:=false "$@" )
amrmpc()    ( _amr_env && exec ros2 launch amr_mpc torque_experiment.launch.py plant:=real use_rviz:=false "$@" )
amr8()      ( _amr_env && exec ros2 launch amr_mpc torque_experiment.launch.py plant:=real use_rviz:=false origin_mode:=robot "$@" )
amrcheck()  ( _amr_env && cd "$AMR_MPC_DIR" && exec /usr/bin/python3 tools/check_localization.py --apply "$@" )
amrmove()   ( _amr_env && cd "$AMR_MPC_DIR" && exec /usr/bin/python3 tools/basic_motion_test.py "$@" )
amrmu()     ( _amr_env && cd "$AMR_MPC_DIR" && exec /usr/bin/python3 tools/mu_ramp_test.py "$@" )   # 2026-11-16 바닥 μ̂
amrviz()    ( _amr_env && exec ros2 launch amr_mpc torque_viz.launch.py "$@" )
amrall()    ( _amr_env && sudo -v && exec ros2 launch amr_mpc real_all.launch.py "$@" )   # 2026-11-16 터미널 하나로
amrramp()   ( _amr_env && sudo -v && exec ros2 launch amr_mpc torque_ramp.launch.py "$@" )   # 2026-11-17 바닥 토크 램프
amrrec() (   # 이름 생략 가능 — 폴더는 record_all.sh 가 bags/<날짜>/<시분초>[_이름] 로 정한다
  _amr_env && exec bash "$AMR_MPC_DIR/tools/record_all.sh" "${1:-}" "amrrec $*"
)
amrhelp() { sed -n '6,25p' "$AMR_MPC_DIR/tools/amr_aliases.sh" | sed 's/^# \{0,1\}//'; }
