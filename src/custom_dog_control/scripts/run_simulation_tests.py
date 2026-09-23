#!/usr/bin/env python3
"""Run each acceptance scenario in its own headless Gazebo process group."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


SCENARIOS = {
    'motion': 'simulation_motion_test.py',
    'matrix': 'simulation_command_matrix_test.py',
}
READY_MESSAGE = 'Controllers active and Gazebo physics running.'


def stop_process_group(process):
    """Only stop children started by this runner, including surviving Gazebo."""
    if process is None:
        return
    for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()  # Reap the group leader so it cannot remain a zombie.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            continue
        break
    process.wait(timeout=5)


def gazebo_master_uri():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return f'http://127.0.0.1:{sock.getsockname()[1]}'


def run_scenario(name, args, output):
    directory = output / name
    directory.mkdir()
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id),
                       ROS_LOCALHOST_ONLY='1', GAZEBO_MASTER_URI=gazebo_master_uri(),
                       PYTHONUNBUFFERED='1')
    result = {'scenario': name, 'status': 'failed', 'ros_domain_id': args.domain_id,
              'gazebo_master_uri': environment['GAZEBO_MASTER_URI'],
              'logs': str(directory)}
    launch = test = None
    start = time.monotonic()
    try:
        with (directory / 'launch.log').open('w') as launch_log, \
                (directory / 'test.log').open('w') as test_log:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'custom_dog_control', 'gazebo.launch.py',
                 'gui:=false', 'use_rviz:=false', 'start_keyboard:=false'],
                env=environment, stdout=launch_log, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            while READY_MESSAGE not in (directory / 'launch.log').read_text(errors='replace'):
                if launch.poll() is not None:
                    raise RuntimeError(f'Launch exited during startup: {launch.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Controllers did not become ready before startup timeout')
                time.sleep(0.5)
            test = subprocess.Popen(
                ['ros2', 'run', 'custom_dog_control', SCENARIOS[name]],
                env=environment, stdout=test_log, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + args.test_timeout
            while test.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError(f'Launch exited during test: {launch.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Acceptance scenario exceeded test timeout')
                time.sleep(0.5)
            result['returncode'] = test.returncode
            if test.returncode:
                raise RuntimeError(f'Acceptance scenario failed: {test.returncode}')
            result['status'] = 'passed'
    except (OSError, RuntimeError, TimeoutError) as error:
        result['error'] = str(error)
    except KeyboardInterrupt:
        result['status'] = 'interrupted'
        raise
    finally:
        stop_process_group(test)
        stop_process_group(launch)
        result['duration_seconds'] = round(time.monotonic() - start, 2)
        (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=[*SCENARIOS, 'all'], default='all')
    parser.add_argument('--domain-id', type=int, default=97,
                        help='Dedicated ROS domain; must not be used by another robot/test (default: 97)')
    parser.add_argument('--startup-timeout', type=float, default=180)
    parser.add_argument('--test-timeout', type=float, default=180)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    if args.startup_timeout <= 0 or args.test_timeout <= 0:
        parser.error('Timeouts must be positive')
    output = (args.output_dir or Path('log') / 'simulation' /
              datetime.now().strftime('%Y%m%d-%H%M%S-%f')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    scenarios = list(SCENARIOS) if args.scenario == 'all' else [args.scenario]
    results = []
    interrupted = False
    print(f'Acceptance logs: {output}', flush=True)
    try:
        for name in scenarios:
            print(f'Cold start: {name}', flush=True)
            result = run_scenario(name, args, output)
            results.append(result)
            print(f"{name}: {result['status']} ({result['duration_seconds']} s)", flush=True)
            if result['status'] != 'passed':
                print(result.get('error', ''), flush=True)
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        (output / 'summary.json').write_text(json.dumps(
            {'requested': scenarios, 'results': results, 'interrupted': interrupted},
            indent=2) + '\n')
    return 130 if interrupted else (0 if len(results) == len(scenarios) and
                                   all(r['status'] == 'passed' for r in results) else 1)


if __name__ == '__main__':
    raise SystemExit(main())
