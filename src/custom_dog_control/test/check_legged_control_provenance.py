#!/usr/bin/env python3

import hashlib
import pathlib
import sys


REVISION = 'a7f381c0367e98e31c01336e678eef47e304d40d'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def check(package_root):
    errors = []
    manifest = package_root / 'config' / 'legged_control_upstream.sha256'
    for number, line in enumerate(manifest.read_text().splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            expected, relative = line.split(None, 1)
        except ValueError:
            errors.append(f'{manifest}:{number}: malformed line')
            continue
        source = package_root / relative
        if not source.is_file():
            errors.append(f'missing upstream source: {relative}')
        elif digest(source) != expected:
            errors.append(f'upstream source changed without provenance update: {relative}')

    lock = (package_root / 'config' / 'dependencies.lock.yaml').read_text()
    if REVISION not in lock:
        errors.append('legged_control revision is not pinned in dependencies.lock.yaml')

    backend = (package_root / 'src' / 'nmpc' / 'NmpcBackend.cpp').read_text()
    required = (
        '#include <legged_interface/LeggedInterface.h>',
        'std::make_unique<legged::LeggedInterface>',
        'std::make_unique<legged::WeightedWbc>',
    )
    for token in required:
        if token not in backend:
            errors.append(f'legged_control runtime binding is missing: {token}')
    if 'std::make_unique<ocs2::legged_robot::LeggedRobotInterface>' in backend:
        errors.append('runtime NMPC backend regressed to the OCS2 example interface')

    weighted_wbc = (
        package_root / 'third_party' / 'legged_wbc' / 'src' / 'WeightedWbc.cpp'
    ).read_text()
    wbc_algorithm_terms = (
        'formulateFloatingBaseEomTask()',
        'formulateTorqueLimitsTask()',
        'formulateFrictionConeTask()',
        'formulateNoContactMotionTask()',
        'formulateSwingLegTask() * weightSwingLeg_',
        'formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_',
        'formulateContactForceTask(inputDesired) * weightContactForce_',
    )
    for token in wbc_algorithm_terms:
        if token not in weighted_wbc:
            errors.append(f'legged_control WeightedWbc task is missing: {token}')

    if errors:
        for error in errors:
            print(f'ERROR: {error}', file=sys.stderr)
        return 1
    print(
        'legged_control provenance valid: '
        f'revision={REVISION}, runtime=LeggedInterface+WeightedWbc'
    )
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: check_legged_control_provenance.py PACKAGE_ROOT', file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(check(pathlib.Path(sys.argv[1]).resolve()))
