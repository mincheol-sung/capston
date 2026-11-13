# Migration Map

This file records how the previous test workspaces were organized into
`~/2026amr_ws/src/amr_basic_stack`.

## Included From `~/test/amr_jazzy_basic_stack`

Moved under `sensors/`:

```text
src/amr_base          -> sensors/amr_base
src/amr_bringup       -> sensors/amr_bringup
src/amr_description   -> sensors/amr_description
src/amr_laser_tools   -> sensors/amr_laser_tools
src/rplidar_ros       -> sensors/rplidar_ros
roboclaw_motor_test.py -> sensors/tools/roboclaw_motor_test.py
roboclaw_probe.py      -> sensors/tools/roboclaw_probe.py
```

## Included From `~/test/amr_ws`

Moved under `perception/`:

```text
src/amr_interfaces          -> perception/amr_interfaces
src/amr_perception_bringup  -> perception/amr_perception_bringup
src/dynamic_filter          -> perception/dynamic_filter
src/motion_predictor        -> perception/motion_predictor
src/object_tracker          -> perception/object_tracker
src/risk_model              -> perception/risk_model
src/semantic_slam           -> perception/semantic_slam
src/sensor_fusion           -> perception/sensor_fusion
yolo11n.pt                  -> perception/models/yolo11n.pt
yolo11n_openvino_model      -> perception/models/yolo11n_openvino_model
```

Moved under `stack_master/`:

```text
docs/CODE_MAP.md            -> stack_master/docs/ORIGINAL_CODE_MAP.md
scripts/setup_nuc_system.sh -> stack_master/scripts/setup_nuc_system.sh
config/cyclonedds.xml       -> stack_master/config/cyclonedds.xml
```

## New In This Workspace

```text
stack_master/package.xml
stack_master/CMakeLists.txt
stack_master/launch/amr_system.launch.py
planner/
controller/
```

## Intentionally Not Copied

```text
~/test/amr_jazzy_basic_stack/.git
~/test/amr_jazzy_basic_stack/.gitignore
~/test/amr_jazzy_basic_stack/build
~/test/amr_jazzy_basic_stack/install
~/test/amr_jazzy_basic_stack/log

~/test/amr_ws/.git
~/test/amr_ws/.gitignore
~/test/amr_ws/build
~/test/amr_ws/install
~/test/amr_ws/log
```

Generated Python caches such as `__pycache__/` and `*.pyc` were also skipped.

## Launch Notes

`stack_master/launch/amr_system.launch.py` starts with `mock_mode:=false`,
`use_sensors:=true`, and `use_joystick:=true`, matching the real-robot launch
workflow. Use `mock_mode:=true use_sensors:=false use_joystick:=false` for
ROS-only checks without motor, LiDAR, or camera hardware.
