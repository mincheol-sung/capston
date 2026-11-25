#!/bin/bash
# 실행기·기록기는 ../2026-11-23_fric_dhat 것. 최종 비교의 MPC 조건 (보상 끔 + 상한 옮김 + 마찰 모델 + 외란 추정) 에 마찰계수·상한 기준만 바꾼다.
D=$(cd "$(dirname "$0")" && pwd)
F=$D/../2026-11-23_fric_dhat
REC=$F/pf_rec.py DOMBASE=${DOMBASE:-70} $F/runner.sh $D/cases.txt $D/runs 3 \
  shape:=line seg_distance:=1.0 speed:=1.0 accel:=2.5 surface_mu:=0.45 sim_com_height:=0.105 limit_mode:=friction \
  disturbance:=true friction_comp:=false mpc_cap_friction_shift:=true use_fric_model:=true
