#!/usr/bin/env python3
"""Map a prepared OFF/ON pair with identical frozen Cartographer configuration."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from PIL import Image


def run(command, logfile, env, timeout=1800):
    started = time.monotonic()
    with logfile.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        last_report = started
        try:
            while process.poll() is None:
                elapsed = time.monotonic() - started
                if elapsed > timeout:
                    raise TimeoutError(f'Timeout; see {logfile}')
                if time.monotonic() - last_report >= 20.:
                    print(json.dumps({'stage': logfile.name, 'wall_s': round(elapsed, 1)}),
                          flush=True)
                    last_report = time.monotonic()
                time.sleep(.5)
            if process.returncode:
                raise RuntimeError(f'Exit {process.returncode}; see {logfile}')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    return {'command': command, 'wall_s': time.monotonic() - started, 'exit_code': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--config-dir', type=Path, required=True)
    parser.add_argument('--repeat-off', action='store_true',
                        help='Add an OFF control run to an existing completed pair')
    args = parser.parse_args()
    output = args.input / 'mapping'
    output.mkdir(exist_ok=args.repeat_off)
    config = output / 'config'
    config.mkdir(exist_ok=args.repeat_off)
    hashes = {}
    for filename in ('mapping_2d.lua', 'mapping_hq_2d.lua'):
        source = args.config_dir / filename
        if not args.repeat_off:
            shutil.copyfile(source, config / filename)
        elif source.read_bytes() != (config / filename).read_bytes():
            raise ValueError('Source configuration changed since the comparison')
        hashes[filename] = hashlib.sha256(source.read_bytes()).hexdigest()
    report = {'config_sha256': hashes, 'runs': {}}
    if args.repeat_off:
        report = json.loads((output / 'run_summary.json').read_text())
    variants = ('off_repeat',) if args.repeat_off else ('off', 'on')
    for index, variant in enumerate(variants):
        env = dict(os.environ, ROS_DOMAIN_ID=str(195 + index),
                   ROS_LOG_DIR=str(output / ('ros_logs_' + variant)))
        stem = output / variant
        if Path(str(stem) + '.pbstream').exists():
            raise ValueError('Refusing to overwrite a previous mapping run')
        command = ['/opt/ros/jazzy/lib/cartographer_ros/cartographer_offline_node',
                   '-configuration_directory', str(config),
                   '-configuration_basenames', 'mapping_hq_2d.lua',
                   '-bag_filenames', str(args.input / ('off' if args.repeat_off else variant)),
                   '-save_state_filename', str(stem) + '.pbstream',
                   '--ros-args', '-p', 'use_sim_time:=true']
        report['runs'][variant] = run(command, output / f'{variant}.log', env)
        command = ['/opt/ros/jazzy/lib/cartographer_ros/cartographer_pbstream_to_ros_map',
                   '-pbstream_filename', str(stem) + '.pbstream',
                   '-map_filestem', str(stem), '-resolution', '0.05']
        report['runs'][variant]['render'] = run(command, output / f'{variant}_render.log', env)
        Image.open(str(stem) + '.pgm').save(str(stem) + '.png')
        (output / 'run_summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({'completed': variant, 'state': str(stem) + '.pbstream',
                          'wall_s': report['runs'][variant]['wall_s']}), flush=True)


if __name__ == '__main__':
    main()
