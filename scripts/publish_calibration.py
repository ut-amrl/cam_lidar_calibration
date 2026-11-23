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
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init();node=Node('saved_calibration_publisher')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    publisher=node.create_publisher(CameraInfo,args.camera_info_topic,qos)
    broadcaster=StaticTransformBroadcaster(node)
    camera=bundle['camera_info'];message=CameraInfo()
    message.header.frame_id=bundle['camera_frame']
    message.width=camera['image_width'];message.height=camera['image_height'];message.distortion_model=camera['distortion_model']
    message.k=camera['camera_matrix']['data'];message.d=camera['distortion_coefficients']['data']
    message.r=camera['rectification_matrix']['data'];message.p=camera['projection_matrix']['data']
    transforms=[]
    for entry in bundle['transforms']:
        tf=TransformStamped();tf.header.frame_id=entry['parent_frame'];tf.child_frame_id=entry['child_frame']
        tf.header.stamp=node.get_clock().now().to_msg()
        for key in ('x','y','z'):setattr(tf.transform.translation,key,float(entry['translation'][key]))
        for key in ('x','y','z','w'):setattr(tf.transform.rotation,key,float(entry['rotation'][key]))
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    def publish():
        message.header.stamp=node.get_clock().now().to_msg();publisher.publish(message)
    publish();timer=node.create_timer(1/args.rate,publish)
    node.get_logger().info(f'Publishing {args.camera_info_topic} and {len(transforms)} static transforms; stop conflicting TF publishers first')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        timer.cancel();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except (ValueError,OSError,KeyError) as error:sys.exit(str(error))
