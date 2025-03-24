#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rclpy
from rclpy.node import Node

import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf_transformations import euler_from_quaternion, quaternion_matrix, inverse_matrix, quaternion_from_matrix, unit_vector

# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
    import python.rrt as rrt
except ImportError:
    raise ImportError(
        'Unable to import potential_field.py. Make sure this file is in "{}"'.format(directory))


SPEED = .2
EPSILON = .1

X = 0
Y = 1
YAW = 2


def get_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))


def feedback_linearized(pose, velocity, epsilon):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # MISSING: Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.

    return u, w


def get_velocity(position, path_points):
    v = np.zeros_like(position)
    if len(path_points) == 0:
        return v
    # Stop moving if the goal is reached.
    if np.linalg.norm(position - path_points[-1]) < .2:
        return v

    # MISSING: Return the velocity needed to follow the
    # path defined by path_points. Assume holonomicity of the
    # point located at position.


    return v


def combine_transforms(trans1: TransformStamped, trans2: TransformStamped) -> TransformStamped:
    combined = TransformStamped()
    combined.header.stamp = trans1.header.stamp
    combined.header.frame_id = trans1.header.frame_id
    combined.child_frame_id = trans2.child_frame_id

    # Combine translations
    combined.transform.translation.x = trans1.transform.translation.x + \
        trans2.transform.translation.x
    combined.transform.translation.y = trans1.transform.translation.y + \
        trans2.transform.translation.y
    combined.transform.translation.z = trans1.transform.translation.z + \
        trans2.transform.translation.z

    # Combine rotations
    q1 = [trans1.transform.rotation.x, trans1.transform.rotation.y,
          trans1.transform.rotation.z, trans1.transform.rotation.w]
    q2 = [trans2.transform.rotation.x, trans2.transform.rotation.y,
          trans2.transform.rotation.z, trans2.transform.rotation.w]
    combined_q = quaternion_multiply(q1, q2)
    combined.transform.rotation.x = combined_q[0]
    combined.transform.rotation.y = combined_q[1]
    combined.transform.rotation.z = combined_q[2]
    combined.transform.rotation.w = combined_q[3]

    return combined


def invert_transform(trans: TransformStamped) -> TransformStamped:
    # Extract translation and rotation
    translation = [trans.transform.translation.x,
                   trans.transform.translation.y,
                   trans.transform.translation.z]
    quaternion = [trans.transform.rotation.x,
                  trans.transform.rotation.y,
                  trans.transform.rotation.z,
                  trans.transform.rotation.w]

    # Convert to transformation matrix
    T = quaternion_matrix(quaternion)
    T[:3, 3] = translation

    # Compute inverse matrix
    T_inv = inverse_matrix(T)

    # Extract inverse translation and quaternion
    inv_translation = T_inv[:3, 3]
    inv_quaternion = quaternion_from_matrix(T_inv)
    inv_quaternion = unit_vector(inv_quaternion)  # Ensure it's normalized

    # Create inverted TransformStamped message
    inverted = TransformStamped()
    inverted.header.stamp = trans.header.stamp  # Consider updating with now()
    inverted.header.frame_id = trans.child_frame_id  # Swap frames
    inverted.child_frame_id = trans.header.frame_id
    inverted.transform.translation.x = inv_translation[0]
    inverted.transform.translation.y = inv_translation[1]
    inverted.transform.translation.z = inv_translation[2]
    inverted.transform.rotation.x = inv_quaternion[0]
    inverted.transform.rotation.y = inv_quaternion[1]
    inverted.transform.rotation.z = inv_quaternion[2]
    inverted.transform.rotation.w = inv_quaternion[3]

    return inverted


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]


class RrtNavigation(Node):
    def __init__(self):
        super().__init__('rrt_navigation')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(
            self)  # for use nav2_bringup to publish map
        self.tf_world_base_link_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self._occupancy_grid = None
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._goal_position = np.array([np.nan, np.nan], dtype=np.float32)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self.path_publisher = self.create_publisher(Path, '/path', 1)

        self.current_path = []
        self.previous_time = self.get_clock().now().to_msg().sec

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.
        self.stop_msg.angular.z = 0.

        self.process_timer = self.create_timer(0.1, self.process_data)

    def map_callback(self, msg):
        values = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.width, msg.info.height))
        processed = np.empty_like(values)
        processed[:] = rrt.FREE
        processed[values < 0] = rrt.UNKNOWN
        processed[values > 50] = rrt.OCCUPIED
        processed = processed.T
        origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
        resolution = msg.info.resolution
        self._occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)

    def goal_callback(self, msg):
        self._goal_position[X] = msg.pose.position.x
        self._goal_position[Y] = msg.pose.position.y
        self.get_logger().info(
            f'Received new goal position: {self._goal_position}')

    def timer_callback(self):
        try:
            # Get the transform from world to odom
            trans_world_odom = self.tf_buffer.lookup_transform(
                'world', 'odom', rclpy.time.Time())
            # Get the transform from odom to base_link
            trans_odom_base_link = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())

            # Combine the transforms to get the transform from world to base_link
            trans_world_base_link = combine_transforms(
                trans_world_odom, trans_odom_base_link)
            self.process_transform(trans_world_base_link)

            # Publish the inverted world to base_link tf as base_link to map,
            # since there is no tf from world to map
            # self.get_logger().info(f'trans_world_base_link: {trans_world_base_link}')
            trans_base_link_map = invert_transform(trans_world_base_link)
            # self.get_logger().info(f'Inverted transform: {trans_base_link_map}')

            trans_base_link_map.header.frame_id = 'base_link'
            trans_base_link_map.child_frame_id = 'map'
            self.tf_broadcaster.sendTransform(trans_base_link_map)
            self.tf_world_base_link_broadcaster.sendTransform(
                trans_world_base_link)

        except Exception as e:
            self.get_logger().info(f'Could not transform: {e}')

    def process_transform(self, trans: TransformStamped):
        self._pose[X] = trans.transform.translation.x
        self._pose[Y] = trans.transform.translation.y
        _, _, yaw = euler_from_quaternion([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w])
        self._pose[YAW] = yaw
        self.get_logger().info(f'Updated pose: {self._pose}')

    def process_data(self):
        current_time = self.get_clock().now().to_msg().sec

        if not self.goal_ready or not self.ready:
            self.get_logger().info(
                f'goal_ready: {self.goal_ready}, ready: {self.ready}')
            return

        goal_reached = np.linalg.norm(self.pose[:2] - self.goal_position) < .2
        if goal_reached:
            self.publisher.publish(self.stop_msg)
            return

        # Follow path using feedback linearization.
        position = np.array([
            self.pose[X] + EPSILON * np.cos(self.pose[YAW]),
            self.pose[Y] + EPSILON * np.sin(self.pose[YAW])], dtype=np.float32)
        v = get_velocity(position, np.array(
            self.current_path, dtype=np.float32))
        u, w = feedback_linearized(self.pose, v, epsilon=EPSILON)
        vel_msg = Twist()
        vel_msg.linear.x = float(u)
        vel_msg.angular.z = float(w)
        self.publisher.publish(vel_msg)

        # Update plan every 1s.
        time_since = current_time - self.previous_time
        if self.current_path and time_since < 2.:
            return
        self.previous_time = current_time

        # Run RRT.
        start_node, final_node = rrt.rrt(
            self.pose, self.goal_position, self.occupancy_grid)
        self.current_path = get_path(final_node)
        if not self.current_path:
            self.get_logger().info(
                f'Unable to reach goal position: {self.goal_position}')

        # Publish path to RViz.
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for u in self.current_path:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = u[X]
            pose_msg.pose.position.y = u[Y]
            path_msg.poses.append(pose_msg)
        self.path_publisher.publish(path_msg)

    @property
    def ready(self):
        return self._occupancy_grid is not None and not np.isnan(self._pose[X])

    @property
    def pose(self):
        return self._pose

    @property
    def occupancy_grid(self):
        return self._occupancy_grid

    @property
    def goal_ready(self):
        return not np.isnan(self._goal_position[0])

    @property
    def goal_position(self):
        return self._goal_position


def get_path(final_node):
    # Construct path from RRT solution.
    if final_node is None:
        return []
    path_reversed = []
    path_reversed.append(final_node)
    while path_reversed[-1].parent is not None:
        path_reversed.append(path_reversed[-1].parent)
    path = list(reversed(path_reversed))
    # Put a point every 5 cm.
    distance = 0.05
    offset = 0.
    points_x = []
    points_y = []
    for u, v in zip(path, path[1:]):
        center, radius = rrt.find_circle(u, v)
        du = u.position - center
        theta1 = np.arctan2(du[1], du[0])
        dv = v.position - center
        theta2 = np.arctan2(dv[1], dv[0])
        # Check if the arc goes clockwise.
        clockwise = np.cross(u.direction, du).item() > 0.
        # Generate a point every 5cm apart.
        da = distance / radius
        offset_a = offset / radius
        if clockwise:
            da = -da
            offset_a = -offset_a
            if theta2 > theta1:
                theta2 -= 2. * np.pi
        else:
            if theta2 < theta1:
                theta2 += 2. * np.pi
        angles = np.arange(theta1 + offset_a, theta2, da)
        offset = distance - (theta2 - angles[-1]) * radius
        points_x.extend(center[X] + np.cos(angles) * radius)
        points_y.extend(center[Y] + np.sin(angles) * radius)
    return list(zip(points_x, points_y))


def main(args=None):
    rclpy.init()
    node = RrtNavigation()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs RRT navigation')
    args, unknown = parser.parse_known_args()
    main(args)
