#!/usr/bin/env python3
"""Stage the canonical description and derive Gazebo's point-foot variant."""

import argparse
import copy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def prepare(source, destination):
    source = source.resolve()
    destination = destination.resolve()
    canonical = source / 'urdf/custom_dog.urdf'
    if not (source / 'package.xml').is_file() or not canonical.is_file():
        raise ValueError(f'Not a custom_dog_description package: {source}')
    tree = ET.parse(canonical)
    original = copy.deepcopy(tree.getroot())
    expected = {f'{leg}_{part}' for leg in ('FR', 'FL', 'RR', 'RL')
                for part in ('thigh', 'calf')}
    removed = set()
    for link in tree.getroot().findall('link'):
        if link.get('name') in expected:
            for collision in list(link.findall('collision')):
                link.remove(collision)
            removed.add(link.get('name'))
    if removed != expected:
        raise ValueError(f'Missing leg links: {expected - removed}')
    # Removing collision elements must be the only model change.
    for root in (original, tree.getroot()):
        for link in root.findall('link'):
            if link.get('name') in expected:
                for collision in list(link.findall('collision')):
                    link.remove(collision)
    if ET.tostring(original) != ET.tostring(tree.getroot()):
        raise ValueError('Point-foot generation changed the model contract')
    if source != destination:
        shutil.copytree(source, destination, dirs_exist_ok=True)
    target = destination / 'urdf/custom_dog_gazebo_point_foot.urdf'
    ET.indent(tree, space='  ')
    tree.write(target, encoding='utf-8', xml_declaration=True)
    print(f'Canonical model: {destination / "urdf/custom_dog.urdf"}')
    print(f'Gazebo point-foot model: {target}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.destination)


if __name__ == '__main__':
    main()
