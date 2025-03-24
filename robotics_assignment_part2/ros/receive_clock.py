#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess


class Clock(Node):
    def __init__(self):
        super().__init__('receive_clock')
        self.subscription = self.create_subscription(String, '/date_time', self.callback, 10)
        self._ready = False

    def callback(self, msg):
        if not self._ready:
            subprocess.call(['sudo date +%F%T -s "{}"'.format(msg.data)], shell=True)
            self.get_logger().info('Time set to {}'.format(msg.data))
            self._ready = True

    @property
    def ready(self):
        return self._ready


def main(args=None):
    rclpy.init(args=args)
    node = Clock()

    # Run sudo once.
    subprocess.call(['sudo echo "Authenticated"'], shell=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if node.ready:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()