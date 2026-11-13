#!/usr/bin/env bash
# All ROS validation tools force isolated localhost domain 170..179 and
# /guide_validation topics. No motor, driver, arming, or gate command is started.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/smoc/2026amr_ws/install/setup.bash
cd /home/smoc/2026amr_ws/src/amr_basic_stack
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export PYTHONPATH="$PWD/planner/local_planner/DRA_MPPI:${PYTHONPATH:-}"
MPPI_PACKAGE="$PWD/planner/local_planner/DRA_MPPI"
VALIDATION_OUTPUT="${1:-/tmp/amr-guide-reproduction}"
mkdir -p "$VALIDATION_OUTPUT"
python3 -m pytest "$MPPI_PACKAGE/test" -q
python3 -m flake8 "$MPPI_PACKAGE/dra_mppi" "$MPPI_PACKAGE/tools" "$MPPI_PACKAGE/test" --select E9,F63,F7,F82,F401,F811
python3 "$MPPI_PACKAGE/tools/validate_mc_accuracy.py" --output "$VALIDATION_OUTPUT/mc_final" --halton --seeds 16 --seed-start 200 --densities 4096 --min-samples 4096
python3 "$MPPI_PACKAGE/tools/validate_mc_domains.py" --output "$VALIDATION_OUTPUT/mc_domains" --seeds 8
python3 "$MPPI_PACKAGE/tools/audit_cost_contributions.py" --output "$VALIDATION_OUTPUT/cost_audit"
python3 "$MPPI_PACKAGE/tools/validate_closed_loop.py" --output "$VALIDATION_OUTPUT/navigation" --rollouts 100 --seeds 3 --seed-start 101 --steps 100
# Short A/B/C/D protocol used in this session:
python3 "$MPPI_PACKAGE/tools/run_ros_comparison.py" --output "$VALIDATION_OUTPUT/ros_abcd" --cycles 100 --repeats 3 --obstacles 10
python3 "$MPPI_PACKAGE/tools/validate_ros_pilot.py" --output "$VALIDATION_OUTPUT/production_dynamic5" --obstacles 5 --cycles 3000 --warmup 50
python3 "$MPPI_PACKAGE/tools/validate_ros_pilot.py" --output "$VALIDATION_OUTPUT/production_static" --obstacles 0 --cycles 300 --warmup 20
python3 "$MPPI_PACKAGE/tools/validate_ros_pilot.py" --output "$VALIDATION_OUTPUT/production_dynamic10" --obstacles 10 --cycles 100 --warmup 10
python3 "$MPPI_PACKAGE/tools/validate_ros_pilot.py" --output "$VALIDATION_OUTPUT/production_dynamic20" --obstacles 20 --cycles 100 --warmup 10
python3 "$MPPI_PACKAGE/tools/validate_ros_pilot.py" --output "$VALIDATION_OUTPUT/standard_dynamic10" --strategy standard --rollouts 100 --timeout 10 --obstacles 10 --cycles 100 --warmup 10
python3 "$MPPI_PACKAGE/tools/summarize_validation.py" --input "$VALIDATION_OUTPUT"
# Requested longer A/B/C/D protocol, NOT executed in full in this session:
# python3 "$MPPI_PACKAGE/tools/run_ros_comparison.py" --output "$VALIDATION_OUTPUT/ros_abcd_long" --cycles 3000 --repeats 3 --obstacles 10
