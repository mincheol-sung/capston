#!/bin/bash
W=$(cd "$(dirname "$0")" && pwd)
COMMON="shape:=square yaw_mode:=tangent square_v_max:=1.5 surface_mu:=0.45"
export REC=$W/diag_rec.py DIAG_OUT=$W/runs DOMBASE=93
$W/runner.sh $W/cases.txt $W/runs 4 $COMMON
for L in 0.2 0.3 0.4; do
  AMR_MPC_TORQUE_BASELINE_A_YAML=$W/pp_L$L.yaml $W/runner.sh $W/cases_L$L.txt $W/runs 2 $COMMON
done
echo ALL_DONE
