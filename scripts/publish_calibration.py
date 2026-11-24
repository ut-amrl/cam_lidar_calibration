#!/usr/bin/env python3
"""Load saved calibration YAML and publish CameraInfo plus /tf_static on demand."""
import argparse
from pathlib import Path
import sys

import numpy as np
import yaml


def load(directory):
    report=yaml.safe_load((directory/'report.yaml').read_text())
    if report.get('status')!='passed':
        raise ValueError('Calibration report did not pass; inspect report.yaml before publishing')
    bundle=yaml.safe_load((directory/'calibration.yaml').read_text())
    info=bundle['camera_info']
    if len(info['camera_matrix']['data'])!=9 or len(info['projection_matrix']['data'])!=12:
        raise ValueError('Malformed camera matrices')
    edges=set();children=set()
    for entry in bundle['transforms']:
        parent,child=entry['parent_frame'],entry['child_frame']
        if not parent or not child or parent==child or child in children:raise ValueError('Invalid or duplicate TF child')
        children.add(child);edges.add((parent,child))
        q=np.array([entry['rotation'][key] for key in ('x','y','z','w')])
        if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:raise ValueError('Invalid transform quaternion')
    return bundle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--camera-info-topic',default='/calibration/camera_info')
    parser.add_argument('--rate',type=float,default=1.)
    parser.add_argument('--dry-run',action='store_true',help='Validate YAML without creating a ROS node')
    args=parser.parse_args()
    bundle=load(args.directory)
    if args.rate<=0:parser.error('Rate must be positive')
    if args.dry_run:
        print(yaml.safe_dump(bundle,sort_keys=False));return 0
    import os
    transforms_file=(args.directory/'robot_transforms.yaml').resolve()
    if not transforms_file.is_file():raise ValueError('Missing robot_transforms.yaml; rerun bag calibration')
    os.execvp('ros2',['ros2','launch','ironback_description','description.launch.py',
                     f'calibration_file:={transforms_file}',f'camera_info_topic:={args.camera_info_topic}',
                     f'camera_info_rate_hz:={args.rate}'])


if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,OSError,KeyError) as error:sys.exit(str(error))
