#!/usr/bin/env python

from __future__ import absolute_import, division, print_function

import rclpy
from rclpy.node import Node
import datetime
from std_msgs.msg import String


class SendClock(Node):
    def __init__(self):
        super().__init__('send_clock')
        self.publisher = self.create_publisher(String, '/date_time', 10)
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1000 Hz

    def timer_callback(self):
        now = self.get_clock().now().to_msg().sec
        date_time = datetime.datetime.fromtimestamp(int(now)).strftime('%Y%m%d %H:%M:%S')
        self.publisher.publish(String(data=date_time))


def main(args=None):
    rclpy.init(args=args)
    node = SendClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()