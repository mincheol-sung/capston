#!/bin/bash
# 실행기·기록기는 ../2026-11-23_fric_dhat 것. 최종 비교의 MPC 조건 (보상 끔 + 상한 옮김 + 마찰 모델 + 외란 추정, μ̂ 0.33 바닥 기준) · 플랜트 구름 손실 5.1 mN·m
D=$(cd "$(dirname "$0")" && pwd)
F=$D/../2026-11-23_fric_dhat
REC=$F/pf_rec.py DOMBASE=${DOMBASE:-70} $F/runner.sh $D/cases.txt $D/runs 3 \
  shape:=line seg_distance:=1.0 limit_mode:=friction disturbance:=true friction_comp:=false mpc_cap_friction_shift:=true use_fric_model:=true \
  mu_hat_wheels:=off mu_hat:=0.33 mpc_cap_on:=ground sim_rolling:=0.0051
