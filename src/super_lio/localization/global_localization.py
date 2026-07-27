#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import copy
import threading
import time

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation

global_map = None
initialized = False
T_map_to_odom = np.eye(4, dtype=np.float64)
cur_odom = None
cur_scan = None
latest_initial_pose = None
first_scan_received = False
_logged_waiting_scan = False
_logged_waiting_pose = False
_T_map_to_odom_valid = False


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


def msg_to_array(pc_msg):
    raw_points = point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
    points = []
    for p in raw_points:
        if hasattr(p, "dtype") and p.dtype.names:
            points.append([p["x"], p["y"], p["z"]])
        else:
            points.append([p[0], p[1], p[2]])
    if len(points) == 0:
        return np.zeros((0, 3))
    return np.asarray(points, dtype=np.float64)


def voxel_down_sample(pcd, voxel_size):
    try:
        return pcd.voxel_down_sample(voxel_size)
    except Exception:
        return o3d.geometry.voxel_down_sample(pcd, voxel_size)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    map_down = voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale)
    result_icp = o3d.pipelines.registration.registration_icp(
        scan_down,
        map_down,
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
    )
    return result_icp.transformation, result_icp.fitness, len(scan_down.points)


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    if pc.size == 0:
        points = []
    else:
        intensity = np.zeros((pc.shape[0], 1), dtype=np.float32)
        points = np.hstack((pc[:, :3].astype(np.float32), intensity)).tolist()
    msg = point_cloud2.create_cloud(header, fields, points)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    if FOV > 3.14:
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )

    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    header = copy.copy(cur_odom.header)
    header.frame_id = MAP_FRAME
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])
    return global_map_in_FOV


def _smooth_transform(current, new, alpha):
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = current[:3, 3].copy() * alpha + new[:3, 3].copy() * (1 - alpha)
    current_rot = current[:3, :3].copy()
    new_rot = new[:3, :3].copy()
    current_axangle = Rotation.from_matrix(current_rot).as_rotvec()
    new_axangle = Rotation.from_matrix(new_rot).as_rotvec()
    smoothed_axangle = current_axangle * alpha + new_axangle * (1 - alpha)
    result[:3, :3] = Rotation.from_rotvec(smoothed_axangle).as_matrix()
    return result


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom, _T_map_to_odom_valid
    node.get_logger().info("Global localization by scan-to-map matching......")
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()
    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)
    transformation, _, _ = registration_at_scale(
        scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
    transformation, fitness, scan_points_fine = registration_at_scale(
        scan_tobe_mapped, global_map_in_FOV, initial=transformation, scale=1)
    node.get_logger().info("Time: {}".format(time.time() - tic))
    node.get_logger().info(
        "Fine ICP: scan points={}, fitness={:.6f}".format(scan_points_fine, fitness))

    if scan_points_fine >= MIN_SCAN_POINTS_FINE and fitness > LOCALIZATION_TH:
        if not _T_map_to_odom_valid:
            T_map_to_odom = transformation
            _T_map_to_odom_valid = True
        elif TRANSFORM_SMOOTHING:
            T_map_to_odom = _smooth_transform(T_map_to_odom, transformation, TRANSFORM_SMOOTHING_ALPHA)
        else:
            T_map_to_odom = transformation
        map_to_odom = Odometry()
        xyz = T_map_to_odom[:3, 3]
        quat = Rotation.from_matrix(np.array(T_map_to_odom[:3, :3], dtype=np.float64, copy=True)).as_quat()
        map_to_odom.pose.pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                                     orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = MAP_FRAME
        pub_map_to_odom.publish(map_to_odom)
        return True

    node.get_logger().warn("Not match!!!!")
    node.get_logger().warn("{}".format(transformation))
    node.get_logger().warn(
        "ICP rejected: scan points={} (min {}), fitness={:.6f} (must be > {:.6f})".format(
            scan_points_fine,
            MIN_SCAN_POINTS_FINE,
            fitness,
            LOCALIZATION_TH,
        ))
    return False


def initialize_global_map(pc_msg):
    global global_map
    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    node.get_logger().info("Global map received.")


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan, first_scan_received
    pc_msg.header.frame_id = ODOM_FRAME
    if cur_odom is not None:
        pc_msg.header.stamp = cur_odom.header.stamp
    pub_pc_in_map.publish(pc_msg)
    pc = msg_to_array(pc_msg)
    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
    if not first_scan_received:
        first_scan_received = True


_last_pose_log_time = 0.0

def cb_save_initial_pose(msg):
    global latest_initial_pose, _last_pose_log_time
    latest_initial_pose = msg
    now = time.time()
    if now - _last_pose_log_time > 0.6:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        rpy = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler("xyz")
        node.get_logger().info(
            "Initial pose received: x={:.3f}, y={:.3f}, z={:.3f}, "
            "roll={:.3f}, pitch={:.3f}, yaw={:.3f}".format(
                pos.x, pos.y, pos.z, rpy[0], rpy[1], rpy[2]
            )
        )
        _last_pose_log_time = now


def thread_localization():
    global T_map_to_odom
    while rclpy.ok():
        time.sleep(1.0 / FREQ_LOCALIZATION)
        if cur_odom is not None and cur_scan is not None:
            tic = time.time()
            global_localization(T_map_to_odom)
            node.get_logger().info("Localization fusion time: {:.3f}s".format(time.time() - tic))


class Waiter:
    def __init__(self):
        self.msg = None
        self.event = threading.Event()

    def cb(self, msg):
        self.msg = msg
        self.event.set()


def wait_for_message(topic, msg_type, qos_profile=1):
    waiter = Waiter()
    sub = node.create_subscription(msg_type, topic, waiter.cb, qos_profile)
    while rclpy.ok() and not waiter.event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return waiter.msg


def get_param(name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


if __name__ == "__main__":
    rclpy.init()
    node = Node("super_lio")
    node.get_logger().info("Localization Node Inited...")

    MAP_VOXEL_SIZE = get_param("localization.map_voxel_size", 0.4)
    SCAN_VOXEL_SIZE = get_param("localization.scan_voxel_size", 0.1)
    FREQ_LOCALIZATION = get_param("localization.frequency", 0.5)
    LOCALIZATION_TH = get_param("localization.fitness_threshold", 0.90)
    MIN_SCAN_POINTS_FINE = get_param("localization.min_scan_points_fine", 2000)
    TRANSFORM_SMOOTHING = get_param("localization.transform_smoothing", True)
    TRANSFORM_SMOOTHING_ALPHA = get_param("localization.transform_smoothing_alpha", 0.7)
    FOV = get_param("localization.fov", 6.244)
    FOV_FAR = get_param("localization.fov_far", 150.0)

    MAP_TOPIC = get_param("localization.map_topic", "/map")
    REGISTERED_SCAN_TOPIC = get_param("localization.registered_scan_topic", "/cloud_registered")
    ODOMETRY_TOPIC = get_param("localization.odometry_topic", "/Odometry")
    INITIAL_POSE_TOPIC = get_param("localization.initial_pose_topic", "/initialpose")
    MAP_TO_ODOM_TOPIC = get_param("localization.map_to_odom_topic", "/map_to_odom")
    CURRENT_SCAN_VIS_TOPIC = get_param("localization.current_scan_vis_topic", "/cur_scan_in_map")
    SUBMAP_VIS_TOPIC = get_param("localization.submap_vis_topic", "/submap")
    MAP_FRAME = get_param("localization.map_frame", "map")
    ODOM_FRAME = get_param("localization.odom_frame", "camera_init")

    pub_pc_in_map = node.create_publisher(PointCloud2, CURRENT_SCAN_VIS_TOPIC, 1)
    pub_submap = node.create_publisher(PointCloud2, SUBMAP_VIS_TOPIC, 1)
    pub_map_to_odom = node.create_publisher(Odometry, MAP_TO_ODOM_TOPIC, 1)

    node.create_subscription(PointCloud2, REGISTERED_SCAN_TOPIC, cb_save_cur_scan, 1)
    node.create_subscription(Odometry, ODOMETRY_TOPIC, cb_save_cur_odom, 1)

    node.get_logger().warn("Waiting for global map......")
    map_qos = QoSProfile(depth=1)
    map_qos.reliability = ReliabilityPolicy.RELIABLE
    map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    initialize_global_map(wait_for_message(MAP_TOPIC, PointCloud2, map_qos))

    node.create_subscription(
        PoseWithCovarianceStamped,
        INITIAL_POSE_TOPIC,
        cb_save_initial_pose,
        10
    )

    while rclpy.ok() and not initialized:
        if not first_scan_received:
            if not _logged_waiting_scan:
                node.get_logger().warn("Waiting for first scan....")
                _logged_waiting_scan = True
            rclpy.spin_once(node, timeout_sec=0.5)
            continue
        if latest_initial_pose is None:
            if not _logged_waiting_pose:
                node.get_logger().warn("Waiting for initial pose....")
                _logged_waiting_pose = True
            rclpy.spin_once(node, timeout_sec=0.5)
            continue
        _logged_waiting_scan = False
        _logged_waiting_pose = False
        initial_pose = pose_to_mat(latest_initial_pose)
        latest_initial_pose = None
        initialized = global_localization(initial_pose)

    node.get_logger().info("Initialize successfully!!!!!!")
    threading.Thread(target=thread_localization, daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()
