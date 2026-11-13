#!/usr/bin/env python3
"""Render recorded AMCL startup, keeping the bag's original receive-time axis.

No ROS nodes or robot drivers are started. A recorded prior map, actual particle
clouds, AMCL covariances, scans and odometry are shown; AMCL is not rerun.
"""

import argparse
import bisect
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from capture_slam_video import static_transform
from render_slam_video import elapsed_label, fit_text, font


WIDTH, HEIGHT = 1920, 1080
BG = (240, 244, 248)
NAVY = (15, 32, 51)
INK = (24, 43, 62)
MUTED = (91, 108, 125)
CYAN = (0, 143, 173)
ORANGE = (218, 118, 14)
RED = (219, 63, 76)
GREEN = (27, 139, 91)
UNKNOWN = (231, 237, 241)
WHITE = (255, 255, 255)
VIEW = (64, 218, 1152, 970)
MAP_CARD = (40, 169, 1176, 996)
SIDE_CARD = (1200, 169, 1880, 996)
F = {n: font(n) for n in [18, 20, 21, 22, 23, 24, 26, 28, 30, 32]}
B = {n: font(n, True) for n in [20, 22, 24, 26, 28, 30, 32, 36, 48, 52]}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(quaternion):
    q = quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Series:
    def __init__(self):
        self.times = []
        self.values = []

    def add(self, timestamp, value):
        self.times.append(timestamp)
        self.values.append(value)

    def last(self, timestamp):
        index = bisect.bisect_right(self.times, timestamp) - 1
        return (index, self.values[index]) if index >= 0 else (-1, None)


class RecordedStartup:
    def __init__(self, bag, end):
        self.bag = bag
        self.cloud = Series()
        self.amcl = Series()
        self.odom = Series()
        self.scans = Series()
        self.events = []
        self.map = None
        transforms = []
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        metadata = reader.get_metadata()
        self.start = metadata.starting_time.nanoseconds / 1e9
        topics = ['/map', '/tf_static', '/particle_cloud', '/amcl_pose',
                  '/odometry/filtered', '/scan', '/initialpose', '/rosout']
        classes = {topic.name: get_message(topic.type)
                   for topic in reader.get_all_topics_and_types()
                   if topic.name in topics}
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
        counts = {topic: 0 for topic in classes}
        while reader.has_next():
            topic, data, record_ns = reader.read_next()
            elapsed = record_ns / 1e9 - self.start
            if elapsed > end:
                break
            msg = deserialize_message(data, classes[topic])
            counts[topic] += 1
            if topic == '/map' and self.map is None:
                self.map = np.asarray(msg.data, np.int8).reshape(
                    msg.info.height, msg.info.width)
                self.resolution = msg.info.resolution
                self.origin = [msg.info.origin.position.x, msg.info.origin.position.y,
                               yaw_of(msg.info.origin.orientation)]
            elif topic == '/tf_static':
                for transform in msg.transforms:
                    t, q = transform.transform.translation, transform.transform.rotation
                    transforms.append({'parent': transform.header.frame_id,
                                       'child': transform.child_frame_id,
                                       'translation': [t.x, t.y, t.z],
                                       'rotation': [q.x, q.y, q.z, q.w]})
            elif topic == '/particle_cloud':
                if msg.header.frame_id != 'map':
                    raise ValueError(f'Unexpected particle frame {msg.header.frame_id}')
                particles = np.array([[p.pose.position.x, p.pose.position.y,
                                       yaw_of(p.pose.orientation), p.weight]
                                      for p in msg.particles], dtype=float)
                self.cloud.add(elapsed, particles)
            elif topic == '/amcl_pose':
                if msg.header.frame_id != 'map':
                    raise ValueError(f'Unexpected AMCL frame {msg.header.frame_id}')
                p = msg.pose.pose
                covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
                self.amcl.add(elapsed, {
                    'header_t': seconds(msg.header.stamp) - self.start,
                    'xyyaw': np.array([p.position.x, p.position.y, yaw_of(p.orientation)]),
                    'std': np.sqrt(np.maximum(0, covariance.diagonal()[[0, 1, 5]])),
                    'cov_xy': covariance[:2, :2].copy()})
            elif topic == '/odometry/filtered':
                p, v = msg.pose.pose, msg.twist.twist
                self.odom.add(elapsed, np.array([
                    seconds(msg.header.stamp) - self.start,
                    p.position.x, p.position.y, yaw_of(p.orientation),
                    v.linear.x, v.linear.y, v.angular.z]))
            elif topic == '/scan':
                r = np.asarray(msg.ranges, float)
                angles = msg.angle_min + np.arange(len(r)) * msg.angle_increment
                mask = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
                points = np.column_stack([r[mask] * np.cos(angles[mask]),
                                          r[mask] * np.sin(angles[mask]),
                                          np.zeros(mask.sum()), np.ones(mask.sum())])
                self.scans.add(elapsed, {
                    'header_t': seconds(msg.header.stamp) - self.start,
                    'frame': msg.header.frame_id, 'points': points})
            elif topic == '/initialpose':
                p = msg.pose.pose
                self.events.append({'t': elapsed, 'kind': 'initialpose',
                                    'xy': [p.position.x, p.position.y]})
            elif topic == '/rosout':
                kind = None
                if msg.name == 'amcl_bootstrap_node':
                    if '전역 측위 요청 완료' in msg.msg:
                        kind = 'global_search'
                    elif 'AMCL 수렴. 초기 위치' in msg.msg:
                        kind = 'accepted'
                    elif '검증: 정합' in msg.msg:
                        kind = 'verification'
                elif msg.name == 'amcl_wiggle_node':
                    if msg.msg.startswith('구간 ') and '제자리 회전' in msg.msg:
                        kind = 'rotation_start'
                    elif 'wiggle 종료' in msg.msg:
                        kind = 'rotation_end'
                if kind:
                    self.events.append({'t': elapsed, 'kind': kind, 'message': msg.msg})
        if self.map is None or not self.cloud.values or not self.amcl.values or not self.odom.values:
            raise ValueError('Missing map, particle cloud, AMCL pose or odometry')
        self.odom_values = np.asarray(self.odom.values)
        self.odom_values[:, 3] = np.unwrap(self.odom_values[:, 3])
        self.odom_header_times = self.odom_values[:, 0]
        if np.any(np.diff(self.odom_header_times) < 0):
            raise ValueError('Odometry header timestamps moved backwards')
        self.scan_frames = {}
        for scan in self.scans.values:
            frame = scan['frame']
            if frame not in self.scan_frames:
                self.scan_frames[frame] = static_transform(transforms, 'base_footprint', frame)
            scan['points'] = (scan['points'] @ self.scan_frames[frame].T)[:, :2]
        accepted = [event['t'] for event in self.events if event['kind'] == 'accepted']
        for event in self.events:
            if event['kind'] == 'initialpose':
                event['kind'] = ('accepted_pose' if any(abs(event['t'] - t) < .4 for t in accepted)
                                 else 'manual_pose')
        self.events.sort(key=lambda event: event['t'])
        self.counts = counts
        print(json.dumps({'source': str(bag), 'counts': counts,
                          'events': self.events}, ensure_ascii=False), flush=True)

    def odom_at_header(self, timestamp):
        index = max(0, int(np.searchsorted(self.odom_header_times, timestamp, side='right')) - 1)
        return self.odom_values[index]

    def hypothesis(self, elapsed, header_t=None):
        _, amcl = self.amcl.last(elapsed)
        odom_index, _ = self.odom.last(elapsed)
        if amcl is None or odom_index < 0:
            return None, amcl
        current = (self.odom_values[odom_index] if header_t is None else
                   self.odom_at_header(min(header_t, self.odom_values[odom_index, 0])))
        anchor = self.odom_at_header(amcl['header_t'])
        xy, yaw = amcl['xyyaw'][:2], amcl['xyyaw'][2]
        angle = yaw - anchor[3]
        c, s = math.cos(angle), math.sin(angle)
        delta = current[1:3] - anchor[1:3]
        propagated = xy + np.array([[c, -s], [s, c]]) @ delta
        return np.array([*propagated, yaw + current[3] - anchor[3]]), amcl


class Renderer:
    def __init__(self, data):
        self.data = data
        self.vw, self.vh = VIEW[2] - VIEW[0], VIEW[3] - VIEW[1]
        h, w = data.map.shape
        theta = data.origin[2]
        c, s = math.cos(theta), math.sin(theta)
        corners = np.array([[0, 0], [w, 0], [w, h], [0, h]]) * data.resolution
        corners = corners @ np.array([[c, s], [-s, c]]) + data.origin[:2]
        low, high = corners.min(axis=0), corners.max(axis=0)
        self.scale = min(self.vw / (high[0] - low[0] + .7),
                         self.vh / (high[1] - low[1] + .7))
        self.xmin = (low[0] + high[0]) / 2 - self.vw / (2 * self.scale)
        self.ymax = (low[1] + high[1]) / 2 + self.vh / (2 * self.scale)
        known = data.map >= 0
        p = np.clip(data.map.astype(float), 0, 100) / 100
        raster = np.empty((*data.map.shape, 3), dtype=np.uint8)
        for channel in range(3):
            raster[:, :, channel] = np.where(known, 255 * (1 - p) + 30 * p, UNKNOWN[channel])
        source = Image.fromarray(np.flipud(raster).copy())
        u = self.scale * data.resolution
        forward = np.array([[u*c, u*s], [-u*s, u*c]])
        offset = [self.scale*(data.origin[0]-self.xmin-data.resolution*h*s),
                  self.scale*(self.ymax-data.origin[1]-data.resolution*h*c)]
        inverse = np.linalg.inv(forward)
        shift = -inverse @ offset
        self.map_image = source.transform((self.vw, self.vh), Image.Transform.AFFINE,
            (*inverse[0], shift[0], *inverse[1], shift[1]),
            resample=Image.Resampling.NEAREST, fillcolor=UNKNOWN)
        self.cloud_cache = -2
        self.cloud_image = self.map_image
        self.base = self.make_base()

    def pixels(self, xy):
        xy = np.asarray(xy)
        return np.column_stack([(xy[:, 0] - self.xmin) * self.scale,
                                (self.ymax - xy[:, 1]) * self.scale])

    def pixel(self, xy):
        return tuple(self.pixels(np.asarray(xy).reshape(1, 2))[0])

    def make_base(self):
        image = Image.new('RGB', (WIDTH, HEIGHT), BG)
        d = ImageDraw.Draw(image)
        d.rectangle((0, 0, WIDTH, 142), fill=NAVY)
        d.text((48, 30), 'AMCL Initial Localization', font=B[52], fill=WHITE)
        d.text((51, 98), 'Recorded startup  /  known map + particle hypotheses + LiDAR',
               font=F[26], fill=(187, 205, 220))
        d.rounded_rectangle((1534, 38, 1869, 102), radius=15, fill=(35, 60, 81))
        d.text((1560, 57), '1x ORIGINAL SPEED', font=B[26], fill=WHITE)
        for box in [MAP_CARD, SIDE_CARD]:
            d.rounded_rectangle(box, radius=18, fill=WHITE, outline=(217, 226, 233), width=2)
        d.text((65, 185), 'Known map / localization hypotheses', font=B[26], fill=INK)
        d.text((1226, 189), 'SOURCE TIME', font=B[20], fill=MUTED)
        d.text((48, 1022), f'Source: {self.data.bag.name}', font=F[23], fill=MUTED)
        d.text((1175, 1022), 'Recorded results  |  1920 x 1080  |  30 fps', font=F[23], fill=MUTED)
        return image

    def cloud_layer(self, elapsed):
        index, cloud = self.data.cloud.last(elapsed)
        if index == self.cloud_cache:
            return self.cloud_image
        self.cloud_cache = index
        if cloud is None:
            self.cloud_image = self.map_image
            return self.cloud_image
        image = np.array(self.map_image)
        overlay = image.copy()
        points = np.rint(self.pixels(cloud[:, :2])).astype(int)
        inside = ((points[:, 0] >= 0) & (points[:, 0] < self.vw) &
                  (points[:, 1] >= 0) & (points[:, 1] < self.vh))
        points = points[inside]
        overlay[points[:, 1], points[:, 0]] = ORANGE
        image = cv2.addWeighted(overlay, .85, image, .15, 0)
        # A bounded subset of orientation ticks keeps large initial clouds legible.
        stride = max(1, len(cloud) // 500)
        for pose in cloud[::stride]:
            x, y = self.pixel(pose[:2])
            if 0 <= x < self.vw and 0 <= y < self.vh:
                direction = pose[2]
                cv2.line(image, (round(x), round(y)),
                         (round(x + 5*math.cos(direction)), round(y - 5*math.sin(direction))),
                         ORANGE, 1, cv2.LINE_AA)
        self.cloud_image = Image.fromarray(image)
        return self.cloud_image

    def phase(self, elapsed):
        events = [e for e in self.data.events if e['t'] <= elapsed]
        kinds = {e['kind'] for e in events}
        odom_index, _ = self.data.odom.last(elapsed)
        moving = odom_index >= 0 and np.linalg.norm(self.data.odom_values[odom_index, 4:6]) > .03
        if 'accepted' in kinds:
            return 'INITIAL POSE ACCEPTED', GREEN
        if 'manual_pose' in kinds:
            return ('ADDITIONAL MOVEMENT' if moving else 'MANUAL POSE ADJUSTMENT'), ORANGE
        if 'rotation_end' in kinds:
            return 'AWAITING CONVERGENCE', ORANGE
        if 'rotation_start' in kinds:
            return 'ROTATING IN PLACE', CYAN
        return ('GLOBAL PARTICLE SEARCH' if 'global_search' in kinds else 'INITIALIZING'), MUTED

    def chart(self, d, elapsed):
        left, top, right, bottom = 1260, 670, 1852, 798
        d.text((1226, 630), 'POSITION UNCERTAINTY HISTORY', font=B[20], fill=MUTED)
        d.text((1690, 630), 'sigma x', font=F[18], fill=CYAN)
        d.text((1781, 630), 'sigma y', font=F[18], fill=ORANGE)
        def py(value):
            return bottom - np.clip((math.log10(max(.08, value)) - math.log10(.08)) /
                                    math.log10(4/.08), 0, 1)*(bottom-top)
        for value in [.1, .3, 1, 3]:
            y = py(value)
            d.line((left, y, right, y), fill=(227, 233, 238), width=1)
            d.text((1226, y-10), f'{value:g}', font=F[18], fill=MUTED)
        d.line((left, py(.3), right, py(.3)), fill=(103, 168, 139), width=2)
        d.text((right-114, py(.3)-22), '0.30 m limit', font=F[18], fill=GREEN)
        count = bisect.bisect_right(self.data.amcl.times, elapsed)
        for axis, color in [(0, CYAN), (1, ORANGE)]:
            path = [(left + self.data.amcl.times[i]/100*(right-left),
                     py(self.data.amcl.values[i]['std'][axis])) for i in range(count)]
            if len(path) > 1:
                d.line(path, fill=color, width=3)
        for value in [0, 25, 50, 75, 100]:
            x = left + value/100*(right-left)
            d.text((x-10, bottom+5), str(value), font=F[18], fill=MUTED)
        x = left + elapsed/100*(right-left)
        d.line((x, top, x, bottom), fill=(122, 137, 150), width=1)

    def frame(self, elapsed):
        image = self.base.copy()
        view = self.cloud_layer(elapsed).copy()
        pose, amcl = self.data.hypothesis(elapsed)
        # The scan is projected at the current AMCL hypothesis, using odometry
        # at the scan/AMCL header times to compensate their acquisition delay.
        _, scan = self.data.scans.last(elapsed)
        if scan is not None and pose is not None and elapsed-self.data.scans.times[
                bisect.bisect_right(self.data.scans.times, elapsed)-1] < .35:
            scan_pose, _ = self.data.hypothesis(elapsed, scan['header_t'])
            c, s = math.cos(scan_pose[2]), math.sin(scan_pose[2])
            xy = scan['points'] @ np.array([[c, s], [-s, c]]) + scan_pose[:2]
            points = np.rint(self.pixels(xy)).astype(int)
            a = np.array(view)
            keep = ((points[:, 0] >= 0) & (points[:, 0] < self.vw) &
                    (points[:, 1] >= 0) & (points[:, 1] < self.vh))
            points = points[keep]
            a[points[:, 1], points[:, 0]] = RED
            view = Image.fromarray(a)
        v = ImageDraw.Draw(view)
        if pose is not None:
            x, y = self.pixel(pose[:2])
            values, vectors = np.linalg.eigh(amcl['cov_xy'])
            angles = np.linspace(0, 2*math.pi, 80)
            ellipse = np.column_stack([np.cos(angles), np.sin(angles)]) @ (
                vectors @ np.diag(np.sqrt(np.maximum(0, values)))).T + pose[:2]
            v.line([tuple(p) for p in self.pixels(ellipse)], fill=CYAN, width=2)
            v.ellipse((x-9, y-9, x+9, y+9), fill=CYAN, outline=WHITE, width=2)
            heading = -pose[2]
            arrow = [(x+26*math.cos(heading), y+26*math.sin(heading))] + [
                (x+12*math.cos(heading+a), y+12*math.sin(heading+a)) for a in [-1, 1]]
            v.polygon(arrow, fill=CYAN, outline=WHITE, width=2)
        # Map legend uses an opaque panel so early wide particle clouds stay legible.
        v.rounded_rectangle((12, 12, 535, 113), radius=10, fill=WHITE)
        for row, (text, color) in enumerate([
            ('Orange: AMCL particle hypotheses', ORANGE),
            ('Cyan: AMCL estimate / 1-sigma ellipse', CYAN),
            ('Red: LiDAR at the position hypothesis', RED)]):
            v.text((24, 21+row*29), text, font=F[20], fill=color)
        length = 2*self.scale
        y = self.vh-29
        v.rounded_rectangle((14, y-45, 35+length, y+12), radius=7, fill=WHITE)
        v.text((24, y-40), '2 m', font=F[20], fill=INK)
        v.line((24, y, 24+length, y), fill=INK, width=3)
        v.line((24, y-6, 24, y+6), fill=INK, width=2)
        v.line((24+length, y-6, 24+length, y+6), fill=INK, width=2)
        image.paste(view, VIEW[:2])
        d = ImageDraw.Draw(image)
        d.text((1224, 217), elapsed_label(elapsed), font=B[52], fill=INK)
        phase, color = self.phase(elapsed)
        d.rounded_rectangle((1226, 291, 1852, 337), radius=9,
                            fill=tuple(int(.90*255+.10*c) for c in color))
        d.text((1240, 300), phase, font=B[24], fill=color)
        index, cloud = self.data.cloud.last(elapsed)
        d.text((1226, 356), f'Particles: {len(cloud):,}' if cloud is not None else 'Particles: waiting',
               font=B[26], fill=ORANGE)
        odom_index, _ = self.data.odom.last(elapsed)
        if odom_index >= 0:
            odom = self.data.odom_values[odom_index]
            angle = odom[3] - self.data.odom_values[0, 3]
            travel = float(np.linalg.norm(np.diff(self.data.odom_values[:odom_index+1, 1:3], axis=0),
                                          axis=1).sum())
            center = (1750, 423)
            d.ellipse((center[0]-48, center[1]-48, center[0]+48, center[1]+48),
                      outline=(214, 227, 235), width=5)
            direction = -angle-math.pi/2
            tip = (center[0]+39*math.cos(direction), center[1]+39*math.sin(direction))
            d.line((center, tip), fill=CYAN, width=6)
            d.ellipse((center[0]-5, center[1]-5, center[0]+5, center[1]+5), fill=CYAN)
            d.text((1226, 400), f'Relative rotation: {math.degrees(angle):.0f} deg', font=F[26], fill=INK)
            d.text((1226, 442), f'Odom travel: {travel:.2f} m', font=F[23], fill=MUTED)
            d.text((1699, 478), 'ODOM YAW', font=F[18], fill=MUTED)
        d.line((1226, 512, 1852, 512), fill=(222, 231, 237), width=2)
        if amcl is not None:
            sx, sy, syaw = amcl['std']
            d.text((1226, 532), f'Position sigma: x {sx:.2f} m  /  y {sy:.2f} m', font=B[24], fill=INK)
            d.text((1226, 574), f'Yaw sigma: {math.degrees(syaw):.1f} deg', font=B[24], fill=INK)
            d.text((1625, 579), 'Limits: 0.30 m / 15 deg', font=F[18], fill=MUTED)
        self.chart(d, elapsed)
        d.line((1226, 837, 1852, 837), fill=(222, 231, 237), width=2)
        d.text((1226, 852), 'RECORDED EVENTS', font=B[20], fill=MUTED)
        names = {'global_search': 'Global particles initialized',
                 'rotation_start': 'In-place rotation started',
                 'rotation_end': 'Rotation finished; still awaiting convergence',
                 'manual_pose': 'Manual initial-pose input',
                 'accepted': 'AMCL accepted; initial pose sent'}
        events = [e for e in self.data.events if e['t'] <= elapsed and e['kind'] in names][-3:]
        for row, event in enumerate(events):
            text = f'{event["t"]:5.1f}s  {names[event["kind"]]}'
            d.text((1226, 884+row*29), fit_text(d, text, F[20], 625), font=F[20],
                   fill=GREEN if event['kind'] == 'accepted' else INK)
        return image


def write_videos(data, output, ends, fps):
    output.mkdir(parents=True, exist_ok=True)
    renderer = Renderer(data)
    writers = []
    for end in ends:
        path = output / f'amcl_initial_search_0_{end:g}s.mp4'
        if path.exists():
            raise FileExistsError(f'Existing output: {path}')
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'avc1'), fps, (WIDTH, HEIGHT))
        if not writer.isOpened():
            raise RuntimeError('H.264 encoder unavailable')
        writers.append((end, path, writer))
    max_frames = round(max(ends)*fps)
    try:
        for index in range(max_frames):
            elapsed = index/fps
            frame = renderer.frame(elapsed)
            encoded = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
            for end, _, writer in writers:
                if index < round(end*fps):
                    writer.write(encoded)
            if index % round(5*fps) == 0 or index == max_frames-1:
                print(f'Rendered {index+1}/{max_frames}; source {elapsed:.2f}s', flush=True)
    finally:
        for _, _, writer in writers:
            writer.release()
    reports = []
    for end, path, _ in writers:
        cap = cv2.VideoCapture(str(path))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        read_fps = cap.get(cv2.CAP_PROP_FPS)
        for index in [0, count//2, count-1]:
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            if not ok or frame.shape[:2] != (HEIGHT, WIDTH):
                raise RuntimeError(f'Cannot decode {path.name}, frame {index}')
        cap.release()
        if count != round(end*fps) or abs(read_fps-fps) > .01:
            raise RuntimeError(f'Incorrect clip duration: {path.name}')
        renderer.frame(end-1/fps).save(path.with_suffix('.poster.png'))
        reports.append({'file': path.name, 'source_start_sec': 0, 'source_end_sec': end,
                        'duration_sec': count/read_fps, 'frames': count, 'fps': read_fps,
                        'resolution': [WIDTH, HEIGHT], 'codec': 'H.264 / avc1', 'speed': 1,
                        'size_bytes': path.stat().st_size})
    times = [1, 10, 20, 28, 64, 72, 98.2, 99.9]
    sheet = Image.new('RGB', (1920, 1080), BG)
    for index, elapsed in enumerate(times):
        frame = renderer.frame(elapsed)
        frame.resize((480, 270), Image.Resampling.LANCZOS).save(
            output / f'preview_{elapsed:g}s.png')
        sheet.paste(frame.resize((480, 270), Image.Resampling.LANCZOS),
                    ((index % 4)*480, (index//4)*270))
    # Two large end-state images below the timeline thumbnails.
    for index, elapsed in enumerate([29.9, 99.9]):
        sheet.paste(renderer.frame(elapsed).resize((960, 540), Image.Resampling.LANCZOS),
                    (index*960, 540))
    sheet.save(output / 'contact_sheet.png')
    report = {'source_bag': str(data.bag), 'bag_start_sec': data.start,
              'method': 'visualization of recorded AMCL outputs; no algorithm rerun',
              'time_axis': 'original bag receive time, 1x speed; no opening/ending holds',
              'map': 'recorded /map prior occupancy grid',
              'particle_cloud': 'recorded /particle_cloud; no fabricated particles',
              'scan_projection': 'AMCL hypothesis propagated with recorded odometry at scan acquisition time',
              'events': data.events, 'topic_counts': data.counts, 'videos': reports}
    (output / 'video_info.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(reports, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--previews-only', action='store_true')
    args = parser.parse_args()
    data = RecordedStartup(args.bag, 100)
    if args.previews_only:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        renderer = Renderer(data)
        for elapsed in [1, 10, 20, 28, 64, 72, 98.2, 99.9]:
            renderer.frame(elapsed).save(args.output_dir / f'preview_{elapsed:g}s.png')
    else:
        write_videos(data, args.output_dir, [30, 100], args.fps)


if __name__ == '__main__':
    main()
