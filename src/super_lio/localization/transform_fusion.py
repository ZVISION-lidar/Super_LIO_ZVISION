#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import copy
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

cur_odom_to_baselink = None
cur_map_to_odom = None
last_tf_stamp = None
path_in_map = None
path_started = False


def pose_to_mat(pose_msg):
    mat = np.eye(4)
    mat[:3, 3] = [
        pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        pose_msg.pose.pose.position.z,
    ]
    mat[:3, :3] = Rotation.from_quat([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w,
    ]).as_matrix()
    return mat


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom, last_tf_stamp, path_in_map, path_started
    br = TransformBroadcaster(node)
    if path_in_map is None:
        path_in_map = Path()
        path_in_map.header.frame_id = MAP_FRAME
    while rclpy.ok():
        time.sleep(1.0 / FREQ_PUB_LOCALIZATION)
        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_odom is None:
            continue

        tf_stamp = cur_odom.header.stamp
        stamp_tuple = (tf_stamp.sec, tf_stamp.nanosec)
        if last_tf_stamp is not None and stamp_tuple == last_tf_stamp:
            continue
        if last_tf_stamp is not None and stamp_tuple < last_tf_stamp:
            node.get_logger().warn("Skip old map->odom TF stamp")
            continue
        last_tf_stamp = stamp_tuple

        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        if not path_started and cur_map_to_odom is not None:
            path_started = True

        xyz = T_map_to_odom[:3, 3]
        quat = Rotation.from_matrix(np.array(T_map_to_odom[:3, :3], dtype=np.float64, copy=True)).as_quat()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = tf_stamp
        tf_msg.header.frame_id = MAP_FRAME
        tf_msg.child_frame_id = ODOM_FRAME
        tf_msg.transform.translation.x = xyz[0]
        tf_msg.transform.translation.y = xyz[1]
        tf_msg.transform.translation.z = xyz[2]
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        br.sendTransform(tf_msg)

        localization = Odometry()
        T_odom_to_base_link = pose_to_mat(cur_odom)
        T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
        xyz = T_map_to_base_link[:3, 3]
        quat = Rotation.from_matrix(np.array(T_map_to_base_link[:3, :3], dtype=np.float64, copy=True)).as_quat()
        localization.pose.pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                                      orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        localization.twist = cur_odom.twist
        localization.header.stamp = cur_odom.header.stamp
        localization.header.frame_id = MAP_FRAME
        localization.child_frame_id = BODY_FRAME
        pub_localization.publish(localization)

        if path_started:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = cur_odom.header.stamp
            pose_stamped.header.frame_id = MAP_FRAME
            pose_stamped.pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                                     orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
            path_in_map.poses.append(pose_stamped)
            if len(path_in_map.poses) > PATH_MAX_LENGTH:
                path_in_map.poses = path_in_map.poses[-PATH_MAX_LENGTH:]
            pub_path_in_map.publish(path_in_map)


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


def get_param(name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


if __name__ == "__main__":
    rclpy.init()
    node = Node("transform_fusion")
    node.get_logger().info("Transform Fusion Node Inited...")

    FREQ_PUB_LOCALIZATION = get_param("transform_fusion.publish_frequency", 50.0)
    ODOMETRY_TOPIC = get_param("transform_fusion.odometry_topic", "/Odometry")
    MAP_TO_ODOM_TOPIC = get_param("transform_fusion.map_to_odom_topic", "/map_to_odom")
    LOCALIZATION_TOPIC = get_param("transform_fusion.localization_topic", "/localization")
    PATH_IN_MAP_TOPIC = get_param("transform_fusion.path_in_map_topic", "/path_in_map")
    PATH_MAX_LENGTH = get_param("transform_fusion.path_max_length", 10000)
    MAP_FRAME = get_param("transform_fusion.map_frame", "map")
    ODOM_FRAME = get_param("transform_fusion.odom_frame", "camera_init")
    BODY_FRAME = get_param("transform_fusion.body_frame", "body")

    node.create_subscription(Odometry, ODOMETRY_TOPIC, cb_save_cur_odom, 1)
    node.create_subscription(Odometry, MAP_TO_ODOM_TOPIC, cb_save_map_to_odom, 1)
    pub_localization = node.create_publisher(Odometry, LOCALIZATION_TOPIC, 1)
    pub_path_in_map = node.create_publisher(Path, PATH_IN_MAP_TOPIC, 1)
    threading.Thread(target=transform_fusion, daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()
