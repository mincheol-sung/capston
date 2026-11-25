#!/bin/bash
# 후보 P1~P3 × A·C, 시뮬 바닥 μ 0.45 (실측 바닥 μ ≈ 0.43~0.48). C = 엔코더 속도 + 새 d̂ + 슬립 가드 + 새 μ̂
D=/home/smoc/capston/capston/analysis/torque_mpc_sim/2026-11-22_fig8_speedmod
B="shape:=circles8 width:=2.0 yaw_mode:=tangent laps:=2 accel:=1.0 speed_mod_phase_deg:=-90 surface_mu:=0.45"
CC="limit_mode:=friction velocity_source:=wheels disturbance:=true slip_guard:=true"
for P in "P1 speed:=0.6 speed_mod_amp:=0.5 speed_mod_cycles:=2" "P2 speed:=0.6 speed_mod_amp:=0.3 speed_mod_cycles:=6" "P3 speed:=0.7 speed_mod_amp:=0.3 speed_mod_cycles:=4"; do
  set -- $P; N=$1; shift
  $D/run_one.sh ${N}_A $B "$@" controller:=pure_pursuit
  $D/run_one.sh ${N}_C $B "$@" $CC
done
echo ALL_DONE
