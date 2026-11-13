from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
from PIL import Image


@dataclass
class RosMap:
    grid_map: np.ndarray
    image: np.ndarray
    image_path: Path
    resolution: float
    origin: tuple
    start_world: Optional[tuple] = None
    goal_world: Optional[tuple] = None

    @property
    def height(self):
        return self.grid_map.shape[0]

    @property
    def width(self):
        return self.grid_map.shape[1]

    def world_to_grid(self, point):
        """Convert ROS map-frame meters to this planner's (x, image_y) pixels."""
        x = int(np.floor((point[0] - self.origin[0]) / self.resolution))
        ros_y = int(np.floor((point[1] - self.origin[1]) / self.resolution))
        image_y = self.height - 1 - ros_y
        return (x, image_y)

    def grid_to_world(self, point):
        """Convert this planner's (x, image_y) pixels back to ROS map-frame meters."""
        ros_y = self.height - 1 - point[1]
        x = self.origin[0] + (point[0] + 0.5) * self.resolution
        y = self.origin[1] + (ros_y + 0.5) * self.resolution
        return (round(float(x), 4), round(float(y), 4))


def load_ros_map(yaml_path, image_path=None, unknown_is_obstacle=True):
    yaml_path = Path(yaml_path)
    with yaml_path.open("r", encoding="utf-8") as f:
        metadata = yaml.safe_load(f)

    image_path = _resolve_image_path(yaml_path, metadata, image_path)
    image = np.array(Image.open(image_path).convert("L"))
    image = _apply_negate(image, metadata.get("negate", 0))

    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    free_thresh = float(metadata.get("free_thresh", 0.25))
    occupancy = (255.0 - image.astype(float)) / 255.0

    if unknown_is_obstacle:
        obstacle_mask = occupancy >= free_thresh
    else:
        obstacle_mask = occupancy >= occupied_thresh

    grid_map = obstacle_mask.astype(int)

    start_world = _read_optional_pose(metadata, "start")
    goal_world = _read_optional_pose(metadata, "goal")

    return RosMap(
        grid_map=grid_map,
        image=image,
        image_path=image_path,
        resolution=float(metadata["resolution"]),
        origin=tuple(metadata["origin"]),
        start_world=start_world,
        goal_world=goal_world,
    )


def find_ros_map_yaml(map_dir, map_name):
    map_dir = Path(map_dir)
    known_names = (
        f"{map_name}.yaml",
        f"{map_name}.yml",
        f"{map_name}.yamml",
        f"{map_name} .yaml",
        f"{map_name} .yml",
        f"{map_name} .yamml",
    )

    map_subdir = map_dir / map_name
    for name in known_names:
        path = map_subdir / name
        if path.is_file():
            return path

    for name in known_names:
        path = map_dir / name
        if path.is_file():
            return path

    candidates = []
    for extension in ("yaml", "yml", "yamml"):
        candidates.extend(map_dir.glob(f"{map_name}*.{extension}"))
        candidates.extend(map_dir.glob(f"{map_name}/{map_name}*.{extension}"))
    candidates = sorted(candidates)
    if not candidates:
        raise FileNotFoundError(f"No YAML file found for {map_name!r} in {map_dir}")

    return max(candidates, key=lambda path: path.stat().st_mtime)


def _apply_negate(image, negate):
    if int(negate) == 0:
        return image
    return 255 - image


def _resolve_image_path(yaml_path, metadata, override_path):
    if override_path:
        path = Path(override_path)
        if path.is_absolute() or path.exists():
            return path
        return yaml_path.parent / path

    path = Path(metadata["image"])
    if path.is_absolute():
        return path
    return yaml_path.parent / path


def _read_optional_pose(metadata, key):
    pose = metadata.get(key)
    if pose is None:
        return None
    return tuple(float(v) for v in pose[:2])
