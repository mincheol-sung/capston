#!/bin/bash
# 2026-11-24 마찰 모델 시뮬. 실행기·기록기는 ../2026-11-23_fric_dhat 것, 공통 조건은 run.sh·run_dhat.sh 와 같다.
D=$(cd "$(dirname "$0")" && pwd)
F=$D/../2026-11-23_fric_dhat
REC=$F/pf_rec.py DOMBASE=${DOMBASE:-70} $F/runner.sh $D/cases_model8.txt $D/runs 3 \
  shape:=circles8 width:=1.6 laps:=2 speed:=0.69 accel:=3.0 speed_mod_amp:=0.5 speed_mod_cycles:=2 speed_mod_phase_deg:=-90 \
  speed_mod_shape:=square yaw_mode:=tangent surface_mu:=0.55 sim_com_height:=0.2 limit_mode:=friction mpc_com_height:=0.2
REC=$F/pf_rec.py DOMBASE=${DOMBASE:-70} $F/runner.sh $D/cases_modelsq.txt $D/runs 3 \
  shape:=square yaw_mode:=tangent accel:=1.5 square_v_max:=1.5 laps:=1 surface_mu:=0.55 sim_com_height:=0.2 limit_mode:=friction mpc_com_height:=0.2
