#!/usr/bin/env python3
import math
import sys
import xml.etree.ElementTree as element_tree


EXPECTED_JOINTS = [
    f'{leg}_{joint}_joint'
    for leg in ('FR', 'FL', 'RR', 'RL')
    for joint in ('hip', 'thigh', 'calf')
]
EXPECTED_FEET = [f'{leg}_foot' for leg in ('FR', 'FL', 'RR', 'RL')]
EXPECTED_COLLISION_LINKS = {
    'base',
    *(f'{leg}_hip' for leg in ('FR', 'FL', 'RR', 'RL')),
    *(f'{leg}_thigh' for leg in ('FR', 'FL', 'RR', 'RL')),
    *(f'{leg}_calf' for leg in ('FR', 'FL', 'RR', 'RL')),
    *EXPECTED_FEET,
}
EXPECTED_MASS = 13.84916


def inertia_is_positive_definite(inertia):
    ixx = float(inertia['ixx'])
    ixy = float(inertia['ixy'])
    ixz = float(inertia['ixz'])
    iyy = float(inertia['iyy'])
    iyz = float(inertia['iyz'])
    izz = float(inertia['izz'])
    minor_two = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    return ixx > 0.0 and minor_two > 0.0 and determinant > 0.0


def check(path):
    root = element_tree.parse(path).getroot()
    errors = []
    links = {link.attrib['name']: link for link in root.findall('link')}
    joints = root.findall('joint')
    actuated = [
        joint for joint in joints if joint.attrib.get('type') != 'fixed'
    ]

    names = [joint.attrib['name'] for joint in actuated]
    if names != EXPECTED_JOINTS:
        errors.append(f'joint order mismatch: {names}')
    for foot in EXPECTED_FEET:
        if foot not in links:
            errors.append(f'missing foot link {foot}')

    total_mass = 0.0
    for name, link in links.items():
        inertial = link.find('inertial')
        if inertial is None:
            errors.append(f'{name} has no inertial')
            continue
        mass = float(inertial.find('mass').attrib['value'])
        if not math.isfinite(mass) or mass <= 0.0:
            errors.append(f'{name} has invalid mass')
        total_mass += mass
        inertia = inertial.find('inertia')
        if inertia is None or not inertia_is_positive_definite(inertia.attrib):
            errors.append(f'{name} inertia is not positive definite')

        collisions = link.findall('collision')
        if name in EXPECTED_COLLISION_LINKS and not collisions:
            errors.append(f'{name} has no collision geometry')
        for collision in collisions:
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                errors.append(f'{name} has invalid collision geometry')
                continue
            shape = geometry[0].tag
            if shape not in {'box', 'cylinder', 'sphere'}:
                errors.append(
                    f'{name} collision must be primitive, got {shape}')
    if abs(total_mass - EXPECTED_MASS) > 1e-5:
        errors.append(
            f'total mass {total_mass:.8f} != {EXPECTED_MASS:.8f}')

    for joint in actuated:
        name = joint.attrib['name']
        axis_node = joint.find('axis')
        limit = joint.find('limit')
        if axis_node is None or limit is None:
            errors.append(f'{name} has no axis or limit')
            continue
        axis = [float(value) for value in axis_node.attrib['xyz'].split()]
        norm = math.sqrt(sum(value * value for value in axis))
        if abs(norm - 1.0) > 1e-4:
            errors.append(f'{name} axis is not normalized')
        if name.endswith('hip_joint') and axis[0] < 0.99:
            errors.append(f'{name} hip axis direction changed')
        if not name.endswith('hip_joint') and axis[1] < 0.99:
            errors.append(f'{name} leg axis direction changed')
        lower = float(limit.attrib['lower'])
        upper = float(limit.attrib['upper'])
        effort = float(limit.attrib['effort'])
        velocity = float(limit.attrib['velocity'])
        if not lower < upper or effort <= 0.0 or velocity <= 0.0:
            errors.append(f'{name} has invalid limits')

    if errors:
        for error in errors:
            print(f'ERROR: {error}', file=sys.stderr)
        return 1
    print(
        f'URDF contract valid: mass={total_mass:.5f} kg, '
        f'joints={len(actuated)}, feet={len(EXPECTED_FEET)}')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: check_urdf_contract.py URDF', file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(check(sys.argv[1]))
