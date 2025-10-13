#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# from twin_bridge.srv import TwinCommand
from twin_bridge.msg import DetectedObject
from twin_bridge.msg import SimpleMsg


class CommandService(Node):
    def __init__(self):
        super().__init__('command_service')

        # Service
        # self.srv = self.create_service(TwinCommand, 'twin_command', self.execute_callback)

        # Publishers registered only (no periodic publishing)
        self.simple_topic_pub = self.create_publisher(SimpleMsg, 'simple_topic', 10)
        self.get_logger().info('Advertised topic: /simple_topic (no publishing)')

        self.detected_object_pub = self.create_publisher(DetectedObject, 'detected_object', 10)
        self.get_logger().info('Advertised topic: /detected_object (no publishing)')

    def execute_callback(self, request, response):
        self.get_logger().info(f"Received command: {request.command}")
        response.success = True
        response.message = f"Command {request.command} received for {request.target_id}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommandService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
