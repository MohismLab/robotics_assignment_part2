#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sys

# Robot motion commands:
# https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html
from geometry_msgs.msg import Twist
# For groundtruth information.
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion

# Import the potential_field.py code rather than copy-pasting.
directory = os.path.join(os.path.dirname(
    os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
    import python.potential_field as potential_field
except ImportError:
    raise ImportError(
        'Unable to import potential_field.py. Make sure this file is in "{}"'.format(directory))


ROBOT_RADIUS = 0.105 / 2.
CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)
CYLINDER_RADIUS = .3 
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
MAX_SPEED = .5
EPSILON = .2

USE_RELATIVE_POSITIONS = False

X = 0
Y = 1
YAW = 2


def get_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))


def feedback_linearized(pose, velocity, epsilon):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # Missing:
    # Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.

    return u, w


def get_relative_position(absolute_pose, absolute_position):
    relative_position = absolute_position.copy()

    # Missing: Compute the relative position of absolute_position in the
    # coordinate frame defined by absolute_pose.

    return relative_position


def get_velocity(point_position, goal_position, obstacle_position):

    # Missing: Implement the potential field method to get the next
    # velocity vector given the point position, goal position and obstacle

    return potential_field.cap(v, max_speed=MAX_SPEED)


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


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]


class PotentialFieldNavigation(Node):
    def __init__(self):
        super().__init__('potential_field_navigation')
        # GroundtruthPose initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

        self.process_timer = self.create_timer(0.1, self.process_data)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_history = []
        with open('/tmp/isaacsim_exercise.txt', 'w'):
            pass

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

    @property
    def groundtruth_ready(self):
        return not np.isnan(self._pose[X])

    @property
    def groundtruth_pose(self):
        return self._pose

    def process_data(self):
        # Make sure all measurements are ready.
        if not self.groundtruth_ready:
            return

        absolute_point_position = np.array([
            self.groundtruth_pose[X] + EPSILON *
            np.cos(self.groundtruth_pose[YAW]),
            self.groundtruth_pose[Y] + EPSILON * np.sin(self.groundtruth_pose[YAW])], dtype=np.float32)

        if USE_RELATIVE_POSITIONS:
            point_position = get_relative_position(
                self.groundtruth_pose, absolute_point_position)
            goal_position = get_relative_position(
                self.groundtruth_pose, GOAL_POSITION)
            obstacle_position = get_relative_position(
                self.groundtruth_pose, CYLINDER_POSITION)
            pose = np.array([0., 0., 0.], dtype=np.float32)
        else:
            point_position = absolute_point_position
            goal_position = GOAL_POSITION
            obstacle_position = CYLINDER_POSITION
            pose = self.groundtruth_pose

        # Get velocity.
        v = get_velocity(point_position, goal_position, obstacle_position)

        u, w = feedback_linearized(pose, v, epsilon=EPSILON)
        vel_msg = Twist()
        vel_msg.linear.x = float(u)
        vel_msg.angular.z = float(w)
        self.publisher.publish(vel_msg)

        # Log groundtruth positions in /tmp/isaacsim_exercise.txt
        self.pose_history.append(np.concatenate(
            [self.groundtruth_pose, absolute_point_position], axis=0))
        if len(self.pose_history) % 10 == 0:
            with open('/tmp/isaacsim_exercise.txt', 'a') as fp:
                fp.write('\n'.join(','.join(str(v) for v in p)
                         for p in self.pose_history) + '\n')
                self.pose_history = []


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigation()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
