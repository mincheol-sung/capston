#!/usr/bin/env python3
"""Run randomized, sequential isolated ROS comparisons in fresh processes.

No actuator launch, arming, or default ROS domain. Default is a short pilot;
--cycles 3000 --repeats 3 is the requested long protocol (about two hours).
Timeout=10s here measures full unsuccessful computation; node still rejects
expired inputs. Production timeout is measured separately without this override.
"""
import argparse
import json
import os
from pathlib import Path
import random
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=100)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--obstacles', type=int, default=10)
    parser.add_argument('--rollouts', type=int, default=100)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    conditions = [('A', 'batch', 1), ('B', 'split', 1), ('C', 'split', 2), ('D', 'split', 5)]
    schedule = [(repeat, *condition) for repeat in range(args.repeats) for condition in conditions]
    random.Random(20261022).shuffle(schedule)
    (args.output/'manifest.json').write_text(json.dumps({
        'schedule': schedule, 'cycles': args.cycles, 'warmup': 10,
        'obstacles': args.obstacles, 'K': args.rollouts, 'N': 10, 'dt': .2,
        'scope': 'planner + DDS fixture, no perception/MPC/robot; frozen nominal',
        'timeout_override': 10., 'density': 4096., 'execution': 'sequential fresh processes',
        'seed': 7, 'shuffle_seed': 20261022}, indent=2))
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for repeat, condition, backend, workers in schedule:
        out = args.output/f'{condition}_repeat{repeat}'
        with (args.output/f'{condition}_repeat{repeat}.log').open('w') as log:
            command = [sys.executable, str(Path(__file__).with_name('validate_ros_pilot.py')),
                       '--output', str(out), '--cycles', str(args.cycles), '--warmup', '10',
                       '--obstacles', str(args.obstacles), '--rollouts', str(args.rollouts),
                       '--backend', backend, '--workers', str(workers), '--timeout', '10', '--frozen']
            subprocess.run(command, env=env, stdout=log, stderr=log, check=True)
        print(f'finished {condition} repeat={repeat}', flush=True)


if __name__ == '__main__':
    main()
