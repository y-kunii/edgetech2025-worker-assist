#!/usr/bin/env python3
"""Simple ROS 2 node for advertising topics used with rosbridge.

This node creates publishers for topics and keeps the node alive.
Currently it does not publish periodically; topics are advertised so that
rosbridge can forward messages from external clients.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from twin_bridge.msg import DetectedObject
from twin_bridge.msg import SimpleMsg


class CommandService(Node):
    """Node that advertises topics for the Digital Twin gateway."""

    def __init__(self) -> None:
        """Initialize the node and create publishers."""
        super().__init__('command_service')

        # Publishers registered only (no periodic publishing)
        self.simple_topic_pub = self.create_publisher(SimpleMsg, 'simple_topic', 10)
        self.get_logger().info('Advertised topic: /simple_topic (no publishing)')

        self.detected_object_pub = self.create_publisher(DetectedObject, 'detected_object', 10)
        self.get_logger().info('Advertised topic: /detected_object (no publishing)')


def main(args: list[str] | None = None) -> None:
    """Entry point that initializes and spins the node."""
    rclpy.init(args=args)
    node = CommandService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

