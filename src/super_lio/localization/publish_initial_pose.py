#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import argparse

import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("roll", type=float)
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()

    rclpy.init()
    node = Node("publish_initial_pose")
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

    quat = Rotation.from_euler("xyz", [args.roll, args.pitch, args.yaw]).as_quat()
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(
        position=Point(x=args.x, y=args.y, z=args.z),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.header.frame_id = args.frame_id
    node.get_logger().info("Initial Pose: {} {} {} {} {} {}".format(
        args.x, args.y, args.z, args.yaw, args.pitch, args.roll))
    for _ in range(5):
        pub_pose.publish(initial_pose)
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
