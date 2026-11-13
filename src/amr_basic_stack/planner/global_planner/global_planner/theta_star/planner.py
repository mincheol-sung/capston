import argparse
import csv
import os
import sys

# --- 폴더 구조 연결 (maps 폴더의 지도 yaml/png를 불러오기 위한 설정) ---
current_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.abspath(os.path.join(current_dir, '..', '..'))


def get_default_map_dir():
    source_maps = os.path.join(package_root, 'maps')
    if os.path.isdir(source_maps):
        return source_maps

    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_maps

    return os.path.join(get_package_share_directory('global_planner'), 'maps')


map_dir = get_default_map_dir()
sys.path.append(package_root)
sys.path.append(map_dir)
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
# -----------------------------------------------------------------

import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

try:
    from global_planner.common.ros_map_loader import find_ros_map_yaml, load_ros_map
    from global_planner.theta_star.costmap import Costmap
    from global_planner.theta_star import theta_star
except ImportError:
    from common.ros_map_loader import find_ros_map_yaml, load_ros_map
    from costmap import Costmap
    import theta_star


OUTPUT_DIR = current_dir


# --- [유틸리티 함수] 경로를 촘촘하게 만들기 (직선 유지, 간격만 쪼갬) ---
def make_dense_path(path, interval=1.0):
    dense_path = []
    for i in range(len(path) - 1):
        p1 = np.array(path[i])
        p2 = np.array(path[i + 1])
        dist = np.linalg.norm(p2 - p1)
        num_points = int(np.ceil(dist / interval))
        for step in range(num_points):
            interpolated_point = p1 + (p2 - p1) * (step / num_points)
            dense_path.append(
                (round(interpolated_point[0], 2), round(interpolated_point[1], 2))
            )
    dense_path.append((float(path[-1][0]), float(path[-1][1])))
    return dense_path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Theta* global path planner for a ROS map YAML and png/pgm image."
        )
    )
    parser.add_argument(
        "map",
        nargs="?",
        default="testmap",
        help="Map name in maps/<map_name>, or a direct path to <map_name>.yaml.",
    )
    parser.add_argument(
        "--map-dir",
        default=map_dir,
        help="Directory to search when the map argument is a map name.",
    )
    parser.add_argument(
        "--image",
        help="Optional png/pgm image path. Overrides the image field in the YAML.",
    )
    parser.add_argument(
        "--start",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Start position in ROS map-frame meters.",
    )
    parser.add_argument(
        "--goal",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Goal position in ROS map-frame meters.",
    )
    parser.add_argument(
        "--start-grid",
        nargs=2,
        type=int,
        metavar=("X", "Y"),
        help="Start position in grid/image pixels.",
    )
    parser.add_argument(
        "--goal-grid",
        nargs=2,
        type=int,
        metavar=("X", "Y"),
        help="Goal position in grid/image pixels.",
    )
    parser.add_argument(
        "--robot-size",
        type=float,
        default=0.30,
        help="Robot safety size in meters for obstacle inflation. Default: 0.30.",
    )
    parser.add_argument(
        "--waypoint-interval",
        type=float,
        default=0.1,
        help="Dense waypoint interval in meters. Default: 0.1.",
    )
    parser.add_argument(
        "--dense-interval",
        type=float,
        help="Deprecated: dense waypoint interval in meters.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Ask for start and goal positions in the terminal. This is the default.",
    )
    parser.add_argument(
        "--use-yaml-pose",
        action="store_true",
        help="Use start and goal from the YAML without asking.",
    )
    parser.add_argument(
        "--show-map",
        action="store_true",
        help="Show the coordinate preview window before asking for points.",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Do not create a coordinate preview image before input.",
    )
    parser.add_argument(
        "--output-dir",
        default=OUTPUT_DIR,
        help="Directory for generated waypoint text files and path images.",
    )
    return parser.parse_args(argv)


def resolve_map_yaml(map_arg, map_directory):
    if map_arg.lower().endswith((".yaml", ".yml", ".yamml")):
        return os.path.abspath(map_arg)
    return find_ros_map_yaml(map_directory, map_arg)


def read_world_point(prompt, default_point=None):
    default_text = ""
    if default_point is not None:
        default_text = f" [{default_point[0]} {default_point[1]}]"

    while True:
        value = input(f"{prompt} x y{default_text}: ").strip()
        if not value and default_point is not None:
            return default_point

        parts = value.replace(",", " ").split()
        if len(parts) != 2:
            print("  x y 형식으로 입력해주세요. 예: -4.45 4.45")
            continue

        try:
            return (float(parts[0]), float(parts[1]))
        except ValueError:
            print("  숫자 좌표를 입력해주세요.")


def get_world_extent(ros_map):
    min_x = ros_map.origin[0]
    max_x = ros_map.origin[0] + ros_map.width * ros_map.resolution
    min_y = ros_map.origin[1]
    max_y = ros_map.origin[1] + ros_map.height * ros_map.resolution
    return (min_x, max_x, min_y, max_y)


def is_world_point_inside_extent(point, extent):
    min_x, max_x, min_y, max_y = extent
    return min_x <= point[0] <= max_x and min_y <= point[1] <= max_y


def draw_coordinate_map(ax, ros_map, costmap=None):
    extent = get_world_extent(ros_map)
    display_map = costmap.inflated_map if costmap is not None else ros_map.grid_map

    ax.imshow(
        np.flipud(display_map),
        cmap='Greys',
        origin='lower',
        extent=extent,
        alpha=0.85,
    )
    ax.set_xlabel('world x [m]')
    ax.set_ylabel('world y [m]')
    ax.set_title('Map Coordinate Preview')
    ax.grid(color='tab:blue', linestyle='--', linewidth=0.5, alpha=0.55)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])

    has_pose_marker = False
    if (
        ros_map.start_world is not None
        and is_world_point_inside_extent(ros_map.start_world, extent)
    ):
        ax.plot(
            ros_map.start_world[0],
            ros_map.start_world[1],
            'go',
            markersize=8,
            label='YAML start',
        )
        has_pose_marker = True
    if (
        ros_map.goal_world is not None
        and is_world_point_inside_extent(ros_map.goal_world, extent)
    ):
        ax.plot(
            ros_map.goal_world[0],
            ros_map.goal_world[1],
            'b*',
            markersize=11,
            label='YAML goal',
        )
        has_pose_marker = True
    if has_pose_marker:
        ax.legend(loc='best')


def save_coordinate_preview(ros_map, costmap, output_dir):
    preview_path = os.path.join(output_dir, 'map_coordinate_preview.png')
    fig, ax = plt.subplots(figsize=(10, 10))
    draw_coordinate_map(ax, ros_map, costmap)
    fig.savefig(preview_path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    return preview_path


def show_coordinate_preview(ros_map, costmap):
    if matplotlib.get_backend().lower() == 'agg':
        print('   - GUI backend이 없어 창으로 표시하지 못했습니다.')
        return

    fig, ax = plt.subplots(figsize=(10, 10))
    draw_coordinate_map(ax, ros_map, costmap)
    print('   - 좌표 확인 후 지도 창을 닫으면 입력을 계속합니다.')
    plt.show()
    plt.close(fig)


def select_positions(args, ros_map):
    if args.start_grid and args.goal_grid:
        return tuple(args.start_grid), tuple(args.goal_grid), None, None

    if args.start_grid or args.goal_grid:
        raise ValueError("--start-grid and --goal-grid must be used together.")

    if args.start and args.goal:
        start_world = tuple(args.start)
        goal_world = tuple(args.goal)
    elif args.start or args.goal:
        raise ValueError("--start and --goal must be used together.")
    elif args.use_yaml_pose:
        if ros_map.start_world is None or ros_map.goal_world is None:
            raise ValueError("YAML does not contain both start and goal poses.")
        start_world = ros_map.start_world
        goal_world = ros_map.goal_world
    else:
        print("출발점/도착점을 ROS map-frame 월드 좌표 x y로 입력하세요.")
        print("좌표는 map_coordinate_preview.png의 x/y 축을 보고 고르면 됩니다.")
        print("픽셀 좌표를 쓰고 싶으면 --start-grid X Y --goal-grid X Y를 사용하세요.")
        start_world = read_world_point("출발점", ros_map.start_world)
        goal_world = read_world_point("도착점", ros_map.goal_world)

    return (
        ros_map.world_to_grid(start_world),
        ros_map.world_to_grid(goal_world),
        start_world,
        goal_world,
    )


def validate_position(name, position, costmap):
    if costmap.is_out_of_bounds(*position):
        raise ValueError(f"{name} is outside the map: {position}")
    if costmap.is_collision(*position):
        raise ValueError(f"{name} is inside an obstacle or safety margin: {position}")


def plan(argv=None):
    args = parse_args(argv)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    for filename in (
        "waypoints.csv",
        "waypoints_grid.csv",
        "waypoints.txt",
        "waypoints_grid.txt",
        "path_result.png",
    ):
        path = os.path.join(output_dir, filename)
        if os.path.exists(path):
            os.remove(path)

    print("1. 지도를 불러오고 코스트맵을 생성합니다...")
    map_yaml = resolve_map_yaml(args.map, args.map_dir)
    print(f"   - map yaml: {map_yaml}")
    ros_map = load_ros_map(map_yaml, image_path=args.image)
    print(f"   - map image: {ros_map.image_path}")
    print(f"   - resolution: {ros_map.resolution} m/cell")
    print(f"   - map size: {ros_map.width} x {ros_map.height} cells")

    raw_map = ros_map.grid_map
    robot_size_m = args.robot_size
    robot_size_cells = int(np.ceil(robot_size_m / ros_map.resolution))
    my_costmap = Costmap(raw_map, robot_size=robot_size_cells)
    print(f"   - robot size: {robot_size_m:.2f} m -> {robot_size_cells} cells")

    positions_are_given = (
        (args.start and args.goal)
        or (args.start_grid and args.goal_grid)
        or args.use_yaml_pose
    )
    if not args.no_preview and not positions_are_given:
        preview_path = save_coordinate_preview(ros_map, my_costmap, output_dir)
        min_x, max_x, min_y, max_y = get_world_extent(ros_map)
        print(f"   - 좌표 preview: {preview_path}")
        print(f"   - world x range: {min_x:.3f} ~ {max_x:.3f}")
        print(f"   - world y range: {min_y:.3f} ~ {max_y:.3f}")
        if args.show_map:
            show_coordinate_preview(ros_map, my_costmap)

    try:
        start_pos, goal_pos, start_world, goal_world = select_positions(args, ros_map)
        validate_position("start", start_pos, my_costmap)
        validate_position("goal", goal_pos, my_costmap)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return None

    print(f"2. 길 찾기 시작! 출발지: {start_pos} -> 목적지: {goal_pos}")
    if start_world is None or goal_world is None:
        print("   - coordinate mode: grid/image pixels")
    else:
        print(f"   - world start: {start_world}, world goal: {goal_world}")
    
    # 4. Theta* 알고리즘 실행 (꺾임점만 있는 원래 경로)
    path = theta_star.theta_star(start_pos, goal_pos, my_costmap)
    
    if path:
        print("길 찾기 성공!")
        
        # --- 📍 좌표값 출력 및 파일 저장 ---
        print("======== [ 최종 경로 좌표 (Waypoints) ] ========")
        # 1. 터미널 출력은 '원래 경로(path)' 사용
        for i, waypoint in enumerate(path):
            print(f" 경유지 {i+1}: X={waypoint[0]}, Y={waypoint[1]}")
        print("================================================")

        # 2. 파일 저장과 시각화는 '촘촘한 경로(dense_path)' 생성 후 사용
        waypoint_interval_m = args.dense_interval or args.waypoint_interval
        dense_interval_cells = waypoint_interval_m / ros_map.resolution
        dense_path = make_dense_path(path, interval=dense_interval_cells)
        dense_world_path = [ros_map.grid_to_world(point) for point in dense_path]
        
        world_waypoints_path = os.path.join(output_dir, "waypoints.csv")
        with open(
            world_waypoints_path, "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.writer(file)
            writer.writerow(("x", "y"))
            writer.writerows(dense_world_path)
        
        # --- 🎨 시각화 코드 ---
        plt.figure(figsize=(10, 10))
        plt.imshow(my_costmap.inflated_map, cmap='Greys', alpha=0.5)
        plt.imshow(raw_map, cmap='Greys', alpha=0.8)
        
        # 시각화 데이터 추출 (촘촘한 경로 사용)
        dx = [p[0] for p in dense_path]
        dy = [p[1] for p in dense_path]
        
        # 빨간 선 그리기
        plt.plot(dx, dy, color='red', linewidth=2.5, label='Theta* Path')
        # 촘촘해진 점들을 주황색으로 표시
        plt.scatter(dx, dy, color='orange', s=5, label='Dense Waypoints', zorder=3)
        
        plt.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start')
        plt.plot(goal_pos[0], goal_pos[1], 'b*', markersize=15, label='Goal')
        
        plt.title('Autonomous Robot Path Planning (Dense Theta*)', fontsize=16)
        plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
        plt.legend()
        
        path_result = os.path.join(output_dir, "path_result.png")
        plt.savefig(path_result, dpi=300, bbox_inches="tight")
        print(f"월드 좌표 웨이포인트: {world_waypoints_path}")
        print(f"경로 이미지: {path_result}")
        
    else:
        print("길을 찾을 수 없습니다.")
        return None

    return {
        "planner": "theta_star",
        "map": args.map,
        "map_yaml": map_yaml,
        "output_dir": output_dir,
        "raw_path_grid": path,
        "dense_path_grid": dense_path,
        "dense_path_world": dense_world_path,
        "start_grid": start_pos,
        "goal_grid": goal_pos,
        "start_world": start_world,
        "goal_world": goal_world,
    }


def main(argv=None):
    result = plan(argv)
    return 0 if result else 1

if __name__ == "__main__":
    main()
