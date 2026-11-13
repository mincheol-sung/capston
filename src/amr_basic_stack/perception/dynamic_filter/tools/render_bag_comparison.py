#!/usr/bin/env python3
"""Render measured scan changes and final Cartographer maps; no synthetic images."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

matplotlib.use('Agg')

BLUE = '#176b87'
RED = '#d74a49'
DARK = '#243746'


def scan_views(root):
    config = json.loads((root / 'filter_summary.json').read_text())['config']
    method = config.get('mask_method', 'radius')
    frames = json.loads((root / 'frames.json').read_text())
    front = [f for f in frames if f['topic'] == '/scan_front_carto']
    rear = [f for f in frames if f['topic'] == '/scan_rear_carto']
    selected = max(front, key=lambda f: f['removed'])
    fig, axes = plt.subplots(1, 2, figsize=(12, 7.8), facecolor='white')
    fig.subplots_adjust(top=.84, bottom=.14, wspace=.16)

    def draw(frame):
        opposite = min(rear, key=lambda f: abs(f['t'] - frame['t']))
        a = np.load(root / 'frames' / frame['file'])
        b = np.load(root / 'frames' / opposite['file'])
        points = np.concatenate([a['points'], b['points']])
        removed = np.concatenate([a['removed'], b['removed']])
        for axis, enabled in zip(axes, (False, True)):
            axis.clear()
            kept = points[~removed] if enabled else points
            axis.scatter(kept[:, 0], kept[:, 1], s=5, c=DARK, alpha=.75,
                         linewidths=0, label='Retained returns' if enabled else 'Original returns')
            if not enabled:
                axis.scatter(points[removed, 0], points[removed, 1], s=10, c=RED,
                             linewidths=0, label='Returns removed by ON')
            for centre, track_id in zip(a['centres'], a['track_ids']):
                if method == 'radius':
                    axis.add_patch(Circle(centre, config['filter_radius'], fill=False,
                                          edgecolor=BLUE, linewidth=1.25, linestyle='--'))
                axis.plot(*centre, '+', color=BLUE, markersize=7)
                axis.text(centre[0]+.08, centre[1]+.37, f'ID {track_id}',
                          color=BLUE, fontsize=8, backgroundcolor='white')
            axis.set(xlim=(-2.1, 2.1), ylim=(-.8, 4.5), aspect='equal',
                     xlabel='odom x [m]', ylabel='odom y [m]')
            axis.grid(alpha=.15)
            axis.spines[['top', 'right']].set_visible(False)
            axis.set_title(f'ON  |  {method} track filter' if enabled
                           else 'OFF  |  existing SLAM input', fontsize=13, loc='left', pad=12)
            axis.legend(loc='lower right', fontsize=8, framealpha=.95)
        fig.suptitle(f'Recorded LiDAR comparison  |  t = {frame["t"]:.2f} s',
                     fontsize=19, fontweight='bold', color=DARK, x=.12, ha='left')
        return points, removed

    _, removed = draw(selected)
    fig.text(.12, .07,
             f'{selected["removed"]} / {selected["valid"]} front returns removed '
             f'({100*selected["removed"]/selected["valid"]:.1f}%) at this instant. '
             'Rear scan is overlaid for context.', fontsize=10, color=DARK)
    fig.text(.12, .035, 'Largest-removal frame within the saved interval. '
             'Crosses: track estimates; dashed circles, if shown: radius gate. No ground truth.',
             fontsize=9, color='#666666')
    fig.savefig(root / 'scan_before_after.png', dpi=180)
    plt.close(fig)

    # All of the preselected interval, including unchanged and expired-track frames.
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 6.4), facecolor='white')
    fig.subplots_adjust(top=.83, bottom=.13, wspace=.17)
    images = []
    for target in np.arange(120., 140., .25):
        frame = min(front, key=lambda f: abs(f['t'] - target))
        draw(frame)
        caption = fig.text(.12, .045, f'Front: {frame["removed"]}/{frame["valid"]} removed  |  '
                           f'status: {frame["reason"]}  |  all times shown, no scene cuts',
                           fontsize=9, color=DARK)
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
        caption.remove()
    images[0].save(root / 'scan_120_140.gif', save_all=True, append_images=images[1:],
                   duration=250, loop=0, optimize=True)
    plt.close(fig)
    rows = list(csv.DictReader((root / 'scan_metrics.csv').open()))
    bins = np.arange(0, 361, 5)
    t = np.asarray([float(r['t']) for r in rows])
    valid = np.asarray([int(r['valid']) for r in rows])
    removed = np.asarray([int(r['removed']) for r in rows])
    hist_valid = np.histogram(t, bins, weights=valid)[0]
    hist_removed = np.histogram(t, bins, weights=removed)[0]
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True, facecolor='white')
    axes[0].bar(bins[:-1]+2.5, 100*hist_removed/np.maximum(hist_valid, 1),
                width=4.3, color=BLUE)
    axes[0].set_ylabel('Removed / valid returns [%]')
    axes[1].plot(t, np.cumsum(removed), color=RED, linewidth=2)
    axes[1].set(xlabel='Time from bag start [s]', ylabel='Cumulative removed returns',
                xlim=(0, 351))
    for axis in axes:
        axis.axvspan(120, 140, color='#f2b544', alpha=.2, label='Visualized interval')
        axis.spines[['top', 'right']].set_visible(False)
        axis.grid(axis='y', alpha=.2)
    axes[0].legend(loc='upper left')
    fig.suptitle('Entire 350.75 s bag  |  62,792 of 12,179,628 returns removed (0.516%)',
                 fontsize=16, fontweight='bold', color=DARK)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(root / 'filter_timeline.png', dpi=170)
    plt.close(fig)
    (root / 'visual_summary.json').write_text(json.dumps({
        'representative_frame': selected,
        'selection': 'Maximum removed points within a 120-140s interval chosen before filtering',
        'gif': 'Continuous 120-140s, 4fps; no omitted time segments'}, indent=2))


def maps(root):
    config = json.loads((root / 'filter_summary.json').read_text())['config']
    method = config.get('mask_method', 'radius')
    data, metrics = {}, {}
    variants = ['off', 'on']
    if (root / 'mapping/off_repeat.pgm').exists():
        variants.append('off_repeat')
    for variant in variants:
        stem = root / 'mapping' / variant
        image = np.asarray(Image.open(str(stem)+'.pgm').convert('L'))
        meta = yaml.safe_load(Path(str(stem)+'.yaml').read_text())
        res, (x, y, _) = meta['resolution'], meta['origin']
        extent = (x, x+image.shape[1]*res, y, y+image.shape[0]*res)
        data[variant] = image, extent
        occupied, free, unknown = image <= 89, image >= 205, image == 128
        intermediate = ~(occupied | free | unknown)
        labels, components = ndimage.label(occupied, np.ones((3, 3)))
        sizes = np.bincount(labels.ravel())[1:]
        metrics[variant] = {
            'resolution': res, 'origin': meta['origin'], 'shape': list(image.shape),
            'occupied_cells': int(occupied.sum()), 'free_area_m2': float(free.sum()*res**2),
            'intermediate_cells': int(intermediate.sum()),
            'intermediate_to_occupied': float(intermediate.sum()/max(1, occupied.sum())),
            'occupied_components': int(components),
            'small_components_lt10cells': int(np.sum(sizes < 10)),
            'occupied_thickness_p90_m': float(np.percentile(
                ndimage.distance_transform_edt(occupied)[occupied], 90)*2*res),
            'warning': 'Whole-map proxies, not ground-truth accuracy or human-removal precision',
        }
    extent = (min(d[1][0] for d in data.values()), max(d[1][1] for d in data.values()),
              min(d[1][2] for d in data.values()), max(d[1][3] for d in data.values()))
    fig, axes = plt.subplots(1, 2, figsize=(15, 8), facecolor='white')
    for axis, variant in zip(axes, ('off', 'on')):
        image, bounds = data[variant]
        axis.imshow(image, cmap='gray', vmin=0, vmax=255, extent=bounds,
                    origin='upper', interpolation='nearest')
        axis.set(xlim=extent[:2], ylim=extent[2:], aspect='equal',
                 xlabel='map x [m]', ylabel='map y [m]')
        axis.set_title(f'{variant.upper()} | ' + ('existing input' if variant == 'off'
                       else f'{method} track filter'), fontsize=15, loc='left')
    fig.suptitle('Cartographer mapping  |  identical bag and mapping_hq_2d.lua',
                 fontsize=20, fontweight='bold', color=DARK)
    fig.text(.1, .035, 'Each map is independently optimized. Same coordinate scale; '
             'no image registration or appearance enhancement. Grid resolution: 5 cm.',
             fontsize=10)
    fig.tight_layout(rect=(0, .07, 1, .93))
    fig.savefig(root / 'maps_before_after.png', dpi=170)
    plt.close(fig)
    (root / 'map_metrics.json').write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--maps-only', action='store_true')
    args = parser.parse_args()
    if not args.maps_only:
        scan_views(args.input)
    if (args.input / 'mapping/on.pgm').exists():
        maps(args.input)


if __name__ == '__main__':
    main()
