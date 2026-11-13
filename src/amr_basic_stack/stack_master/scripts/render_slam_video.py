#!/usr/bin/env python3
"""Render a time-faithful SLAM capture to a presentation-sized MP4.

Input: maps/index.jsonl (t, file), map NPZs (data, resolution, origin),
poses.csv (t,x,y,yaw), and optional metadata.json. Times are source seconds;
occupancy arrays have the ROS grid convention (row zero at the bottom).

Example:
    python3 render_slam_video.py --capture /tmp/slam_capture \
        --output /tmp/cartographer_4x.mp4 --speed 4 --fps 30

Use --speed 1 (or --realtime) on a second run for a real-time version.
Only recorded maps whose timestamp has passed are displayed. The viewport
uses the final map's extent throughout; it never displays the final map early.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 1920, 1080
START_HOLD, END_HOLD = 1.0, 3.0
BACKGROUND = (240, 244, 248)
HEADER = (15, 32, 51)
INK = (24, 43, 62)
MUTED = (92, 110, 128)
ACCENT = (0, 149, 179)
UNKNOWN = (233, 238, 242)
OCCUPIED = (36, 47, 59)
WHITE = (255, 255, 255)
MAP_CARD = (56, 174, 1460, 990)
MAP_VIEW = (78, 236, 1438, 966)
SIDE_CARD = (1484, 174, 1864, 990)
RESAMPLE = getattr(Image, "Resampling", Image)
AFFINE = getattr(Image, "Transform", Image).AFFINE


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    names = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
        if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


FONTS = {size: font(size) for size in (19, 21, 22, 24, 26, 28, 30, 34)}
FONTS_BOLD = {size: font(size, True) for size in (22, 24, 28, 32, 38, 54, 58)}


def elapsed_label(seconds: float, tenths: bool = True) -> str:
    seconds = max(0.0, seconds)
    # Round first so a fractional value cannot be printed as 00:60.0.
    rounded = round(seconds, 1) if tenths else int(seconds)
    minutes, remaining = divmod(rounded, 60)
    return f"{int(minutes):02d}:{remaining:04.1f}" if tenths else f"{int(minutes):02d}:{int(remaining):02d}"


def fit_text(draw: ImageDraw.ImageDraw, value: str, face: Any, width: int) -> str:
    if draw.textlength(value, font=face) <= width:
        return value
    while value and draw.textlength(value + "…", font=face) > width:
        value = value[:-1]
    return value + "…"


def load_grid(path: Path) -> tuple[np.ndarray, float, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        data = np.asarray(archive["data"], dtype=np.int16)
        resolution = float(np.asarray(archive["resolution"]).reshape(-1)[0])
        origin = np.asarray(archive["origin"], dtype=float).reshape(-1)
    if data.ndim != 2 or min(data.shape) == 0:
        raise ValueError(f"Grid must be a nonempty H×W array: {path}")
    if not math.isfinite(resolution) or resolution <= 0 or origin.size < 3:
        raise ValueError(f"Invalid resolution/origin in {path}")
    if not np.isfinite(origin[:3]).all():
        raise ValueError(f"Non-finite origin in {path}")
    return data, resolution, origin[:3]


class Capture:
    def __init__(self, directory: Path):
        self.directory = directory
        metadata_path = directory / "metadata.json"
        self.metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        self.maps: list[tuple[float, Path]] = []
        index = directory / "maps" / "index.jsonl"
        with index.open() as stream:
            for number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                record = json.loads(line)
                timestamp = float(record["t"])
                if not math.isfinite(timestamp):
                    raise ValueError(f"Non-finite timestamp at {index}:{number}")
                path = directory / "maps" / record["file"]
                if not path.is_file():
                    raise FileNotFoundError(path)
                self.maps.append((timestamp, path))
        self.maps.sort(key=lambda item: item[0])
        if not self.maps:
            raise ValueError(f"No maps in {index}")
        self.map_times = np.array([item[0] for item in self.maps], dtype=float)

        rows = []
        poses_path = directory / "poses.csv"
        if poses_path.exists():
            with poses_path.open(newline="") as stream:
                for record in csv.DictReader(stream):
                    row = tuple(float(record[key]) for key in ("t", "x", "y", "yaw"))
                    if np.isfinite(row).all():
                        rows.append(row)
        # Retain the last pose for duplicate timestamps, avoiding zero-length
        # interpolation intervals after replay resets or repeated messages.
        rows.sort(key=lambda row: row[0])
        unique = {row[0]: row for row in rows}
        self.poses = np.array(list(unique.values()), dtype=float).reshape(-1, 4)
        self.pose_times = self.poses[:, 0]
        self.yaw = np.unwrap(self.poses[:, 3]) if len(self.poses) else np.array([])
        first = min(self.map_times[0], self.pose_times[0] if len(rows) else math.inf)
        last = max(self.map_times[-1], self.pose_times[-1] if len(rows) else -math.inf)
        self.start = float(self.metadata.get("bag_start_sec", first))
        # Some capture metadata measures duration from the first selected
        # sensor message, while bag_start_sec refers to the whole bag. Never
        # truncate the observed tail by adding that duration to an earlier
        # start. Prefer the absolute final record when it is provided.
        metadata_duration = float(self.metadata.get("duration_sec", 0.0))
        record_end = float(self.metadata.get("last_record_ns", last * 1e9)) * 1e-9
        self.duration = max(metadata_duration, last - self.start, record_end - self.start)
        if not math.isfinite(self.start) or not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError("Capture start/duration must be finite; duration must be nonnegative")
        if self.start > last or self.start + self.duration < first:
            raise ValueError("metadata timeline does not overlap the recorded maps/poses")

        data, resolution, origin = load_grid(self.maps[-1][1])
        height, width = data.shape
        local = np.array([[0, 0], [width * resolution, 0],
                          [width * resolution, height * resolution], [0, height * resolution]])
        c, s = math.cos(origin[2]), math.sin(origin[2])
        corners = local @ np.array([[c, s], [-s, c]]) + origin[:2]
        points = corners
        if len(self.poses):
            points = np.concatenate([points, self.poses[:, 1:3]], axis=0)
        low, high = points.min(axis=0), points.max(axis=0)
        extent = np.maximum(high - low, resolution)
        padding = max(float(extent.max()) * 0.035, 0.2)
        self.bounds = (low[0] - padding, low[1] - padding,
                       high[0] + padding, high[1] + padding)

    def pose_at(self, timestamp: float) -> tuple[float, float, float] | None:
        if not len(self.poses) or timestamp < self.pose_times[0]:
            return None
        left = int(np.searchsorted(self.pose_times, timestamp, side="right")) - 1
        right = min(left + 1, len(self.poses) - 1)
        if left == right:
            return float(self.poses[left, 1]), float(self.poses[left, 2]), float(self.yaw[left])
        fraction = (timestamp - self.pose_times[left]) / (self.pose_times[right] - self.pose_times[left])
        xy = self.poses[left, 1:3] * (1 - fraction) + self.poses[right, 1:3] * fraction
        heading = self.yaw[left] * (1 - fraction) + self.yaw[right] * fraction
        return float(xy[0]), float(xy[1]), float(heading)


class Renderer:
    def __init__(self, capture: Capture, speed: float):
        self.capture = capture
        self.speed = speed
        self.view_width = MAP_VIEW[2] - MAP_VIEW[0]
        self.view_height = MAP_VIEW[3] - MAP_VIEW[1]
        xmin, ymin, xmax, ymax = capture.bounds
        self.scale = min(self.view_width / (xmax - xmin), self.view_height / (ymax - ymin))
        # Center the fixed final extent in the viewport, preserving metric aspect.
        self.xmin = (xmin + xmax) / 2 - self.view_width / (2 * self.scale)
        self.ymax = (ymin + ymax) / 2 + self.view_height / (2 * self.scale)
        self.pose_pixels = [(self.pixel(x, y)) for x, y in capture.poses[:, 1:3]]
        self.cached_index: int | None = None
        self.cached_map = Image.new("RGB", (self.view_width, self.view_height), UNKNOWN)
        self.base = self.make_base()

    def pixel(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.xmin) * self.scale, (self.ymax - y) * self.scale

    def make_base(self) -> Image.Image:
        canvas = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
        draw = ImageDraw.Draw(canvas)
        draw.rectangle((0, 0, WIDTH, 142), fill=HEADER)
        draw.text((58, 31), "Cartographer 2D SLAM", font=FONTS_BOLD[54], fill=WHITE)
        draw.text((61, 99), "Fresh SLAM from recorded sensors  /  occupancy grid + trajectory",
                  font=FONTS[24], fill=(183, 203, 220))
        draw.rounded_rectangle((1637, 41, 1864, 99), radius=16, fill=(36, 60, 82))
        draw.text((1661, 57), "SLAM REPLAY", font=FONTS_BOLD[22], fill=(213, 232, 244))
        for card in (MAP_CARD, SIDE_CARD):
            draw.rounded_rectangle(card, radius=18, fill=WHITE, outline=(219, 227, 234), width=2)
        draw.text((80, 193), "Map reconstruction", font=FONTS_BOLD[28], fill=INK)
        draw.text((1170, 198), "map frame  ·  metres", font=FONTS[21], fill=MUTED)
        bag = str(self.capture.metadata.get("bag_name", self.capture.directory.name))
        draw.text((60, 1020), fit_text(draw, f"Source: {bag}", FONTS[22], 1340),
                  font=FONTS[22], fill=MUTED)
        draw.text((1503, 1020), "1920 × 1080  |  ROS replay", font=FONTS[22], fill=MUTED)
        return canvas

    def map_at(self, timestamp: float) -> Image.Image:
        index = int(np.searchsorted(self.capture.map_times, timestamp, side="right")) - 1
        if index == self.cached_index:
            return self.cached_map
        self.cached_index = index
        if index < 0:
            self.cached_map = Image.new("RGB", (self.view_width, self.view_height), UNKNOWN)
            return self.cached_map
        data, resolution, origin = load_grid(self.capture.maps[index][1])
        known = data >= 0
        probability = np.clip(data, 0, 100).astype(np.float32) / 100.0
        raster = np.empty((*data.shape, 3), dtype=np.uint8)
        for channel in range(3):
            raster[:, :, channel] = np.where(
                known, np.rint(255 * (1 - probability) + OCCUPIED[channel] * probability),
                UNKNOWN[channel]).astype(np.uint8)
        source = Image.fromarray(np.flipud(raster).copy(), "RGB")
        # Source image row zero is the grid's upper edge. Include origin yaw
        # in the world-to-viewport affine, then invert for Pillow sampling.
        c, s = math.cos(origin[2]), math.sin(origin[2])
        unit = self.scale * resolution
        forward = np.array([[unit * c, unit * s], [-unit * s, unit * c]])
        offset = np.array([
            self.scale * (origin[0] - self.xmin - resolution * data.shape[0] * s),
            self.scale * (self.ymax - origin[1] - resolution * data.shape[0] * c),
        ])
        inverse = np.linalg.inv(forward)
        shift = -inverse @ offset
        affine = (inverse[0, 0], inverse[0, 1], shift[0],
                  inverse[1, 0], inverse[1, 1], shift[1])
        self.cached_map = source.transform(
            (self.view_width, self.view_height), AFFINE, affine,
            resample=RESAMPLE.NEAREST, fillcolor=UNKNOWN)
        return self.cached_map

    def draw_scale(self, draw: ImageDraw.ImageDraw) -> None:
        desired = 150 / self.scale
        power = 10 ** math.floor(math.log10(desired))
        length = max(value * power for value in (0.1, 0.2, 0.5, 1, 2, 5) if value * power <= desired)
        pixels = length * self.scale
        x, y = 23, self.view_height - 34
        label = f"{length:g} m"
        draw.rounded_rectangle((x - 12, y - 45, x + max(pixels, 80) + 15, y + 14),
                               radius=8, fill=WHITE)
        draw.line((x, y, x + pixels, y), fill=INK, width=4)
        draw.line((x, y - 7, x, y + 7), fill=INK, width=3)
        draw.line((x + pixels, y - 7, x + pixels, y + 7), fill=INK, width=3)
        draw.text((x, y - 39), label, font=FONTS[22], fill=INK)

    def draw_side(self, draw: ImageDraw.ImageDraw, elapsed: float,
                  pose: tuple[float, float, float] | None, phase: str) -> None:
        x = 1512
        draw.text((x, 203), "SOURCE ELAPSED", font=FONTS_BOLD[22], fill=MUTED)
        draw.text((x - 2, 244), elapsed_label(elapsed), font=FONTS_BOLD[58], fill=INK)
        draw.text((x, 320), f"of {elapsed_label(self.capture.duration)}", font=FONTS[28], fill=MUTED)
        draw.rounded_rectangle((x, 372, 1835, 380), radius=4, fill=(224, 233, 240))
        fraction = elapsed / self.capture.duration if self.capture.duration else 1.0
        if fraction > 0:
            draw.rectangle((x, 372, x + max(1, 323 * min(1.0, fraction)), 380),
                           fill=ACCENT)
        label = "1× real time" if self.speed == 1 else f"{self.speed:g}× playback"
        draw.text((x, 407), label, font=FONTS_BOLD[32], fill=ACCENT)
        draw.text((x, 455), phase, font=FONTS[22], fill=MUTED)
        draw.line((x, 501, 1835, 501), fill=(226, 233, 239), width=2)
        draw.text((x, 527), "ROBOT POSE", font=FONTS_BOLD[22], fill=MUTED)
        if pose is None:
            draw.text((x, 569), "Waiting for pose…", font=FONTS[24], fill=MUTED)
        else:
            heading = (math.degrees(pose[2]) + 180) % 360 - 180
            draw.text((x, 566), f"x   {pose[0]:8.2f} m", font=FONTS[28], fill=INK)
            draw.text((x, 610), f"y   {pose[1]:8.2f} m", font=FONTS[28], fill=INK)
            draw.text((x, 654), f"yaw {heading:7.1f}°", font=FONTS[28], fill=INK)
        draw.line((x, 710, 1835, 710), fill=(226, 233, 239), width=2)
        draw.text((x, 736), "LEGEND", font=FONTS_BOLD[22], fill=MUTED)
        entries = [("Robot / heading", "robot"), ("Travelled path", "path"),
                   ("Occupied", OCCUPIED), ("Free space", WHITE), ("Unknown", UNKNOWN)]
        for index, (text, symbol) in enumerate(entries):
            y = 785 + index * 37
            if symbol == "robot":
                draw.ellipse((x + 4, y + 3, x + 22, y + 21), fill=ACCENT)
                draw.polygon([(x + 18, y + 2), (x + 32, y + 12), (x + 18, y + 22)], fill=ACCENT)
            elif symbol == "path":
                draw.line((x + 3, y + 12, x + 32, y + 12), fill=ACCENT, width=5)
            else:
                draw.rectangle((x + 4, y + 2, x + 29, y + 25), fill=symbol,
                               outline=(173, 187, 199), width=1)
            draw.text((x + 47, y), text, font=FONTS[22], fill=INK)

    def frame(self, elapsed: float, phase: str = "Bag sensor replay") -> Image.Image:
        elapsed = float(np.clip(elapsed, 0, self.capture.duration))
        timestamp = self.capture.start + elapsed
        canvas = self.base.copy()
        viewport = self.map_at(timestamp).copy()
        draw_map = ImageDraw.Draw(viewport)
        pose = self.capture.pose_at(timestamp)
        count = int(np.searchsorted(self.capture.pose_times, timestamp, side="right"))
        # The path contains only history, plus the current interpolated pose.
        path = self.pose_pixels[:count]
        if pose is not None:
            path = path + [self.pixel(pose[0], pose[1])]
        if len(path) > 1:
            draw_map.line(path, fill=WHITE, width=7, joint="curve")
            draw_map.line(path, fill=ACCENT, width=4, joint="curve")
        if pose is not None:
            px, py = self.pixel(pose[0], pose[1])
            draw_map.ellipse((px - 12, py - 12, px + 12, py + 12), fill=ACCENT, outline=WHITE, width=3)
            heading = -pose[2]
            arrow = [(px + 27 * math.cos(heading), py + 27 * math.sin(heading))]
            arrow.extend((px + 13 * math.cos(heading + angle), py + 13 * math.sin(heading + angle))
                         for angle in (-1.0, 1.0))
            draw_map.polygon(arrow, fill=ACCENT, outline=WHITE, width=2)
        self.draw_scale(draw_map)
        if self.cached_index is not None and self.cached_index < 0:
            draw_map.text((26, 24), "Waiting for first recorded map…", font=FONTS[26], fill=MUTED)
        canvas.paste(viewport, MAP_VIEW[:2])
        self.draw_side(ImageDraw.Draw(canvas), elapsed, pose, phase)
        return canvas


def open_writer(output: Path, fps: float) -> tuple[Any, str]:
    for codec in ("avc1", "mp4v"):
        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*codec), fps, (WIDTH, HEIGHT))
        if writer.isOpened():
            return writer, codec
        writer.release()
    raise RuntimeError("OpenCV could not open an MP4 encoder (tried avc1, mp4v)")


def render(capture: Capture, output: Path, speed: float, fps: float) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    renderer = Renderer(capture, speed)
    duration = START_HOLD + capture.duration / speed + END_HOLD
    frames = max(1, math.ceil(duration * fps))
    writer, codec = open_writer(output, fps)
    print(f"Rendering {frames} frames ({duration:.1f} s, {speed:g}×, {codec}) to {output}", flush=True)
    progress_interval = max(1, round(fps * 5))
    try:
        for index in range(frames):
            video_time = index / fps
            source_elapsed = min(capture.duration, max(0.0, (video_time - START_HOLD) * speed))
            phase = ("Starting bag sensor replay" if video_time < START_HOLD else
                     "Final map · replay complete" if source_elapsed >= capture.duration else
                     "Bag sensor replay")
            frame = renderer.frame(source_elapsed, phase)
            writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
            if index % progress_interval == 0 or index == frames - 1:
                print(f"  {index + 1}/{frames}  source {elapsed_label(source_elapsed)}", flush=True)
    finally:
        writer.release()
    if not output.exists() or output.stat().st_size < 1024:
        raise RuntimeError(f"Encoder did not produce a usable video: {output}")
    poster = output.with_name(output.stem + ".poster.png")
    renderer.frame(capture.duration, "Final map · replay complete").save(poster)
    sheet = Image.new("RGB", (WIDTH, 720), BACKGROUND)
    for index, elapsed in enumerate(np.linspace(0, capture.duration, 6)):
        thumbnail = renderer.frame(float(elapsed)).resize((640, 360), RESAMPLE.LANCZOS)
        sheet.paste(thumbnail, ((index % 3) * 640, (index // 3) * 360))
    contact = output.with_name(output.stem + ".contact_sheet.png")
    sheet.save(contact)
    print(f"Saved video: {output}\nSaved poster: {poster}\nSaved contact sheet: {contact}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--capture", type=Path, required=True, help="Capture directory")
    parser.add_argument("--output", type=Path, required=True, help="Output .mp4 path")
    parser.add_argument("--speed", type=float, default=4.0, help="Source time / video time (default: 4)")
    parser.add_argument("--fps", type=float, default=30.0, help="Output frame rate (default: 30)")
    parser.add_argument("--realtime", action="store_true", help="Equivalent to --speed 1")
    args = parser.parse_args()
    speed = 1.0 if args.realtime else args.speed
    if not math.isfinite(speed) or speed <= 0:
        parser.error("--speed must be finite and positive")
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be finite and positive")
    if args.output.suffix.lower() != ".mp4":
        parser.error("--output must have an .mp4 extension")
    render(Capture(args.capture), args.output, speed, args.fps)


if __name__ == "__main__":
    main()
