from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def depth_camera_include():
    return Node(
        package=LaunchConfiguration("depth_camera_pkg"),
        executable="realsense2_camera_node",
        name=LaunchConfiguration("depth_camera_name"),
        namespace=LaunchConfiguration("depth_camera_namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_depth_camera")),
        parameters=[{
            "camera_name": ParameterValue(LaunchConfiguration("depth_camera_name"), value_type=str),
            "camera_namespace": ParameterValue(LaunchConfiguration("depth_camera_namespace"), value_type=str),
            # 해상도 명시 지정. 기본값(color 1280x720x30 / depth 848x480x30)은 링크가
            # USB3.2 5Gbps라도 UVC isochronous 스케줄링이 못 버텨 "Incomplete video
            # frame detected"(완성도 13~94%)가 5~10초마다 발생 → color 실효 11Hz로 저하.
            # 2026-08-19 실측 확인. 640x480x15로 낮춰 프레임 손상 제거.
            # ※ YOLO 추론부하는 여기서 안 줄어듦(yolo_imgsz=640으로 letterbox되므로 입력
            #   해상도와 무관). CPU 부하를 줄이려면 sensor_fusion 의 yolo_imgsz 를 조정.
            "rgb_camera.color_profile": ParameterValue(
                LaunchConfiguration("depth_camera_color_profile"), value_type=str),
            "depth_module.depth_profile": ParameterValue(
                LaunchConfiguration("depth_camera_depth_profile"), value_type=str),
            # IR 스트림은 쓰지 않으므로 끈다. rs_launch.py 를 거치던 때는 거기서 false 로
            # 지정했는데, Node 직접 실행으로 바뀌면서 노드 내부 기본값(True)으로 떨어졌다.
            # 켜져 있으면 848x480@30 Y8 두 개(각 12.2MB/s)가 USB 대역폭을 먹어
            # "Incomplete video frame detected" 가 재발한다. 2026-08-19 실측 확인.
            "enable_infra1": False,
            "enable_infra2": False,
            "pointcloud.enable": ParameterValue(
                LaunchConfiguration("depth_camera_pointcloud_enable"), value_type=bool),
            # 점군을 켜면 640x480x15 가 그대로 나가 약 147 MB/s 다. CycloneDDS 가
            # 못 버틴다(RGB 2.7MB 프레임 때 이미 rmem_max 를 올려야 했다).
            # decimation 은 드라이버 안에서 depth 를 줄이므로 DDS 를 타기 전에
            # 데이터가 작아진다. magnitude 3 이면 640x480 -> 약 213x160 = 34k 점.
            # ※ 기본값은 꺼짐이다. 켜면 align_depth 결과 해상도도 같이 줄어들어
            #   sensor_fusion 의 roi_median_3d 가 쓰는 정렬 깊이가 바뀐다.
            #   점군을 쓰는 launch 에서만 켤 것.
            "decimation_filter.enable": ParameterValue(
                LaunchConfiguration("depth_camera_decimation_enable"), value_type=bool),
            "decimation_filter.filter_magnitude": ParameterValue(
                LaunchConfiguration("depth_camera_decimation_magnitude"), value_type=int),
            "align_depth.enable": ParameterValue(
                LaunchConfiguration("depth_camera_align_depth"), value_type=bool),
            # D455 내장 IMU -> robot_localization EKF 입력(/camera/imu)
            "enable_gyro": ParameterValue(
                LaunchConfiguration("depth_camera_enable_imu"), value_type=bool),
            "enable_accel": ParameterValue(
                LaunchConfiguration("depth_camera_enable_imu"), value_type=bool),
            "unite_imu_method": ParameterValue(
                LaunchConfiguration("depth_camera_unite_imu_method"), value_type=int),
            # 카메라 프레임(camera_link->optical)은 URDF가 정의하므로 드라이버 TF는 끔
            # (안 끄면 robot_state_publisher와 TF 충돌)
            "publish_tf": ParameterValue(
                LaunchConfiguration("depth_camera_publish_tf"), value_type=bool),
            # 평상시에는 false. 장치가 busy 상태에 고착됐을 때만 복구용으로 true.
            "initial_reset": ParameterValue(
                LaunchConfiguration("depth_camera_initial_reset"), value_type=bool),
        }],
    )


def scan_merger_node():
    return Node(
        package="amr_laser_tools",
        executable="scan_merger",
        name="dual_scan_merger",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_scan_merge")),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("amr_laser_tools"),
                "config",
                "scan_merge.yaml",
            ])
        ],
    )


def cartographer_scan_filter_node(prefix, input_topic, output_topic):
    """Mask self-observation while preserving each lidar's native timing."""
    config = PathJoinSubstitution([
        FindPackageShare("amr_laser_tools"),
        "config",
        "scan_merge.yaml",
    ])
    return Node(
        package="amr_laser_tools",
        executable="scan_sector_filter",
        name=f"{prefix}_cartographer_scan_filter",
        output="screen",
        condition=IfCondition(LaunchConfiguration(f"use_{prefix}_lidar")),
        parameters=[config],
        remappings=[
            ("scan_in", input_topic),
            ("scan_out", output_topic),
        ],
    )


def rplidar_s2_node(prefix, scan_topic):
    return Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name=f"rplidar_{prefix}",
        output="screen",
        condition=IfCondition(LaunchConfiguration(f"use_{prefix}_lidar")),
        parameters=[{
            "channel_type": "serial",
            "serial_port": LaunchConfiguration(f"{prefix}_lidar_serial_port"),
            "serial_baudrate": LaunchConfiguration("rplidar_s2_serial_baudrate"),
            "frame_id": LaunchConfiguration(f"{prefix}_lidar_frame"),
            "inverted": LaunchConfiguration(f"{prefix}_lidar_inverted"),
            "flip_x_axis": True,
            "angle_compensate": LaunchConfiguration("rplidar_s2_angle_compensate"),
            "scan_mode": LaunchConfiguration("rplidar_s2_scan_mode"),
        }],
        # rplidar_node는 'scan'으로 발행 → 앞/뒤 각각 다른 토픽으로 리매핑
        remappings=[("scan", scan_topic)],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        # 라이다 프레임(base_link 대비, URDF 정의). 좌우반전(mirror)은 회전 프레임으로
        # 불가 → inverted 로 처리. 방향(앞뒤)은 URDF rpy_offset(yaw)로 처리.
        DeclareLaunchArgument("front_lidar_frame", default_value="laser_front"),
        DeclareLaunchArgument("rear_lidar_frame", default_value="laser_rear"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),

        DeclareLaunchArgument("use_front_lidar", default_value="true"),
        DeclareLaunchArgument("front_lidar_serial_port", default_value="/dev/lidar_front"),
        DeclareLaunchArgument("front_lidar_inverted", default_value="false"),

        DeclareLaunchArgument("use_rear_lidar", default_value="true"),
        DeclareLaunchArgument("rear_lidar_serial_port", default_value="/dev/lidar_rear"),
        DeclareLaunchArgument("rear_lidar_inverted", default_value="false"),

        DeclareLaunchArgument("rplidar_s2_serial_baudrate", default_value="1000000"),
        DeclareLaunchArgument("rplidar_s2_angle_compensate", default_value="true"),
        DeclareLaunchArgument("rplidar_s2_scan_mode", default_value="DenseBoost"),

        DeclareLaunchArgument("use_scan_merge", default_value="true",
                              description="두 라이다 스캔을 단일 /scan 으로 병합"),

        DeclareLaunchArgument("use_depth_camera", default_value="true"),
        DeclareLaunchArgument("depth_camera_pkg", default_value="realsense2_camera"),
        DeclareLaunchArgument("depth_camera_launch", default_value="rs_launch.py"),
        DeclareLaunchArgument("depth_camera_name", default_value="camera"),
        DeclareLaunchArgument("depth_camera_namespace", default_value="",
                              description="빈 값이면 토픽이 /camera/... (기본 camera면 /camera/camera/... 이중)"),
        # USB 대역폭 여유 위해 낮은 해상도/프레임(프레임 손상 방지). 케이블·포트 좋아지면 올려도 됨.
        DeclareLaunchArgument("depth_camera_depth_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_camera_color_profile", default_value="640x480x15"),
        DeclareLaunchArgument("depth_camera_pointcloud_enable", default_value="false",
                              description="/camera/depth/color/points 발행. decimation 도 같이 켤 것"),
        DeclareLaunchArgument("depth_camera_decimation_enable", default_value="false",
                              description="depth 다운샘플. 점군 대역폭용. align_depth 해상도도 같이 줄어든다"),
        DeclareLaunchArgument("depth_camera_decimation_magnitude", default_value="3",
                              description="decimation 배율. 3 이면 640x480 -> 약 213x160"),
        DeclareLaunchArgument("depth_camera_align_depth", default_value="true",
                              description="depth를 color에 픽셀정렬 (RGB-D 퓨전 필수)"),
        DeclareLaunchArgument("depth_camera_enable_imu", default_value="true",
                              description="D455 내장 IMU(gyro+accel) 활성화 -> /camera/imu"),
        DeclareLaunchArgument("depth_camera_unite_imu_method", default_value="2",
                              description="gyro+accel 결합: 0=none 1=copy 2=linear_interpolation"),
        DeclareLaunchArgument("depth_camera_publish_tf", default_value="false",
                              description="realsense 드라이버 TF 발행 여부. URDF가 카메라 프레임 소유 시 false"),
        DeclareLaunchArgument("depth_camera_initial_reset", default_value="false",
                              description="복구용 장치 HW 리셋. 평상시 false, busy 고착 시에만 true"),

        rplidar_s2_node("front", "/scan_front"),
        rplidar_s2_node("rear", "/scan_rear"),
        cartographer_scan_filter_node(
            "front", "/scan_front", "/scan_front_carto"),
        cartographer_scan_filter_node(
            "rear", "/scan_rear", "/scan_rear_carto"),
        scan_merger_node(),
        depth_camera_include(),
    ])
