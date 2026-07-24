#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import argparse

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def cloud_msg(points, frame_id, stamp):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    if points.size == 0:
        data = []
    else:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        data = np.hstack((points[:, :3].astype(np.float32), intensity)).tolist()
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    msg = point_cloud2.create_cloud(header, fields, data)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--period", type=float, default=5.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node("map_publisher")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(PointCloud2, "/map", qos)
    pcd = o3d.io.read_point_cloud(args.map)
    points = np.asarray(pcd.points)
    node.get_logger().info("Loaded map {} with {} points".format(args.map, len(points)))

    def timer_cb():
        pub.publish(cloud_msg(points, args.frame_id, node.get_clock().now().to_msg()))

    node.create_timer(args.period, timer_cb)
    timer_cb()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
