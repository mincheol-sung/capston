#!/bin/bash
# 더 센 가감속 8자 × A · C (바닥 토크 상한) · C (지금 상한), 시뮬 바닥 μ 0.45
D=/home/smoc/capston/capston/analysis/torque_mpc_sim/2026-11-22_fig8_speedmod
B="shape:=circles8 width:=2.0 yaw_mode:=tangent laps:=2 speed_mod_phase_deg:=-90 surface_mu:=0.45"
CC="limit_mode:=friction velocity_source:=wheels disturbance:=true slip_guard:=true"
for P in "S1 speed:=0.7 speed_mod_amp:=0.5 speed_mod_cycles:=2 accel:=1.0" \
         "S2 speed:=0.6 speed_mod_amp:=0.5 speed_mod_cycles:=2 accel:=1.0 speed_mod_shape:=square" \
         "S3 speed:=0.6 speed_mod_amp:=0.5 speed_mod_cycles:=2 accel:=1.5 speed_mod_shape:=square"; do
  set -- $P; N=$1; shift
  $D/run_one.sh ${N}_A $B "$@" controller:=pure_pursuit
  $D/run_one.sh ${N}_Cg $B "$@" $CC mpc_cap_on:=ground
  $D/run_one.sh ${N}_Ct $B "$@" $CC mpc_cap_on:=total
done
echo ALL_DONE
