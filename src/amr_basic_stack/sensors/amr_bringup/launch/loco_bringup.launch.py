"""LOCO YOLO 검출 + 메카넘 모터 + 조이스틱 통합 런치.

터미널 하나로 아래를 모두 띄운다.
  - robot_state_publisher (URDF TF)
  - roboclaw_mecanum_node (메카넘 모터 드라이버)
  - joy_node + joy_teleop_node (게임패드)
  - realsense2_camera (검증된 컬러 전용 프로파일)
  - loco_detect detector_node (venv 파이썬으로 직접 실행)

라이다는 끈 상태다. 나중에 켜려면 use_front_lidar/use_rear_lidar/use_scan_merge
인자를 true 로 주면 sensors.launch.py 쪽으로 상속돼 들어간다.

사용 예
  ros2 launch amr_bringup loco_bringup.launch.py \
      model_path:=/home/smoc/yolo_bench/loco_seed_compare/models/yolo11n_seed0_best_int8_openvino_model

모델 교체는 model_path 만 바꾸면 된다.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "launch",
        "bringup.launch.py",
    ])
    realsense_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"),
        "launch",
        "rs_launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("amr_bringup"),
        "rviz",
        "loco_detect.rviz",
    ])

    args = [
        # ── 검출기 ──────────────────────────────────────────────
        DeclareLaunchArgument(
            "model_path",
            description="OpenVINO 모델 디렉터리 (절대경로). 필수.",
        ),
        DeclareLaunchArgument(
            "venv_python",
            default_value="/home/smoc/venvs/yolo/bin/python3",
            description="ultralytics 가 설치된 venv 의 파이썬. "
                        "activate 없이 이 바이너리를 직접 부르면 venv 가 적용된다.",
        ),
        DeclareLaunchArgument(
            "detector_script",
            default_value="/home/smoc/test_hw/src/loco_detect/detector_node.py",
            description="loco_detect 는 아직 colcon 패키지로 안 잡혀 있어 소스를 직접 실행한다.",
        ),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("device", default_value="intel:gpu"),
        DeclareLaunchArgument(
            "publish_annotated",
            default_value="true",
            description="RViz2 확인용 박스 그린 이미지. 지연 측정 시 false 로.",
        ),
        DeclareLaunchArgument("log_every", default_value="60"),
        DeclareLaunchArgument("use_detector", default_value="true"),

        # ── RViz2 ───────────────────────────────────────────────
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="기본값은 /loco/image_annotated 만 전체창으로 띄우는 설정.",
        ),

        # ── 카메라 ──────────────────────────────────────────────
        # camera_namespace 를 비우면 토픽이 /camera/color/... (한 번)가 된다.
        # 기본값 'camera' 면 /camera/camera/color/... 로 이중이 되므로
        # detector 의 input_topic 기본값과 여기를 반드시 맞춰야 한다.
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value=""),
        DeclareLaunchArgument(
            "color_profile",
            default_value="848x480x30",
            description="검증된 프로파일. USB 대역폭 여유가 없으면 낮출 것.",
        ),
        DeclareLaunchArgument(
            "enable_depth",
            default_value="false",
            description="지금은 2D 검출만 쓰므로 꺼서 USB 대역폭을 아낀다.",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/camera/color/image_raw",
            description="camera_namespace 를 바꾸면 여기도 같이 바꿔야 한다.",
        ),

        # ── 베이스 ──────────────────────────────────────────────
        DeclareLaunchArgument(
            "mock_mode",
            default_value="false",
            description="true 면 시리얼을 안 열고 로그만 찍는다. 모터가 안 돈다.",
        ),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("use_front_lidar", default_value="false"),
        DeclareLaunchArgument("use_rear_lidar", default_value="false"),
        DeclareLaunchArgument("use_scan_merge", default_value="false"),
    ]

    # 모터 + 조이스틱 + URDF TF.
    # 카메라는 여기서 끄고 아래에서 검증된 인자로 직접 띄운다.
    # (sensors.launch.py 는 depth/align_depth/IMU 를 켜고 color_profile 을
    #  전달하지 않아 지금 돌아가는 구성과 다르다.)
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "mock_mode": LaunchConfiguration("mock_mode"),
            "use_joystick": LaunchConfiguration("use_joystick"),
            "use_front_lidar": LaunchConfiguration("use_front_lidar"),
            "use_rear_lidar": LaunchConfiguration("use_rear_lidar"),
            "use_depth_camera": "false",
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "camera_namespace": LaunchConfiguration("camera_namespace"),
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "enable_depth": LaunchConfiguration("enable_depth"),
            # 카메라 프레임은 URDF(amr_basic.urdf)가 소유한다.
            # 여기서 true 면 robot_state_publisher 와 TF 가 충돌한다.
            "publish_tf": "false",
            # 매 시작마다 USB/HID를 재바인딩하면 IMU 초기화와 경합하므로 기본 비활성화.
            # 장치가 실제로 busy 상태에 고착됐을 때만 true로 복구 실행한다.
            "initial_reset": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    # Node() 가 아니라 ExecuteProcess 인 이유:
    # ultralytics 는 venv 에만 있는데 Node() 는 설치된 실행파일을 시스템
    # 파이썬으로 부른다. venv 파이썬을 직접 지정해야 import 가 된다.
    detector = ExecuteProcess(
        cmd=[
            LaunchConfiguration("venv_python"),
            LaunchConfiguration("detector_script"),
            "--ros-args",
            "-r", "__node:=loco_detector",
            "-p", ["model_path:=", LaunchConfiguration("model_path")],
            "-p", ["input_topic:=", LaunchConfiguration("input_topic")],
            "-p", ["conf:=", LaunchConfiguration("conf")],
            "-p", ["imgsz:=", LaunchConfiguration("imgsz")],
            "-p", ["device:=", LaunchConfiguration("device")],
            "-p", ["publish_annotated:=", LaunchConfiguration("publish_annotated")],
            "-p", ["log_every:=", LaunchConfiguration("log_every")],
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("use_detector")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",  # RViz 로그는 시끄러워서 검출기 통계를 가린다
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [base, camera, detector, rviz])
