#!/usr/bin/env bash
# Source the ROS underlay and newly built interfaces overlay before running.
set -euo pipefail
semantic_repo_root=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
cd "$semantic_repo_root"
export ROS_DOMAIN_ID=83
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export CYCLONEDDS_URI=""
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
if [[ "$OSTYPE" == darwin* ]]; then
  # Native macOS dlopen searches DYLD paths for ROS typesupport plugin names.
  # Set this inside the script: system bash/env can strip inherited DYLD vars.
  semantic_library_paths=$(python -c 'import os; print(":".join(p + "/lib" for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p))')
  export DYLD_FALLBACK_LIBRARY_PATH="$semantic_library_paths:${CONDA_PREFIX:-}/lib:${DYLD_FALLBACK_LIBRARY_PATH:-}"
fi
export PYTHONPATH="$semantic_repo_root/src/amr_basic_stack/perception/semantic_slam:$semantic_repo_root/src/amr_basic_stack/perception/sensor_fusion:$semantic_repo_root/src/amr_basic_stack/perception/object_tracker:$semantic_repo_root/src/amr_basic_stack/perception/traversability:$semantic_repo_root/src/amr_basic_stack/planner/global_planner:${PYTHONPATH:-}"
python -m pytest -q \
  src/amr_basic_stack/perception/semantic_slam/test/test_object_map.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_persistence.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_persistence_node.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_pose_graph.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_cartographer_adapter.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_dynamic_gate.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_ros_pipeline.py \
  src/amr_basic_stack/perception/semantic_slam/test/test_ros_isolation.py \
  src/amr_basic_stack/perception/amr_perception_bringup/test/test_semantic_launch.py \
  src/amr_basic_stack/perception/sensor_fusion/test/test_semantic_geometry.py \
  src/amr_basic_stack/perception/sensor_fusion/test/test_semantic_detector.py \
  src/amr_basic_stack/perception/sensor_fusion/test/test_frame_guard.py \
  src/amr_basic_stack/perception/sensor_fusion/test/test_semantic_output_split.py \
  src/amr_basic_stack/planner/global_planner/test/test_path_validation.py \
  src/amr_basic_stack/planner/global_planner/test/test_traversable_map_adapter.py \
  src/amr_basic_stack/stack_master/test/test_autonomy_semantic_launch.py \
  src/amr_basic_stack/perception/traversability/test
