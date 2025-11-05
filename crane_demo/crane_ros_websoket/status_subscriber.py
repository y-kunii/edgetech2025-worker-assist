import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')
        self.operating_subscription = self.create_subscription(
            String,
            'operating_status_topic',
            self.operating_callback,
            10
        )
        self.gripper_subscription = self.create_subscription(
            String,
            'gripper_status_topic',
            self.gripper_callback,
            10
        )
        self.pick_and_place_publisher = self.create_publisher(
            String,
            'pickand_place_topic',
            10
        )
        self._publish_count = 0
        self._publish_timer = self.create_timer(1.0, self._publish_timer_callback)

    def operating_callback(self, msg):
        self.get_logger().info(f'[Operating Status] {msg.data}')

    def gripper_callback(self, msg):
        self.get_logger().info(f'[Gripper Status] {msg.data}')

    def publish_pickandplace(self, text: str):
        msg = String()
        msg.data = text
        self.pick_and_place_publisher.publish(msg)
        self.get_logger().info(f'[Publish pickand_place_topic] {text}')

    def _publish_timer_callback(self):
        self._publish_count += 1
        self.publish_pickandplace(f'tick {self._publish_count}')

def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
