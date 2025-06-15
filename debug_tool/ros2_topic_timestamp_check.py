#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import argparse
from sensor_msgs.msg import Image, CompressedImage

class MultiSubscriber(Node):
    def __init__(self, topics):
        super().__init__('multi_subscriber')
        self._subscriptions = []
        for topic in topics:
            # Determine message type based on topic name.
            if "compressed" in topic:
                msg_type = CompressedImage
            else:
                msg_type = Image

            sub = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.listener_callback(msg, t),
                10)
            self._subscriptions.append(sub)
            self.get_logger().info(f"Subscribed to topic: {topic} (type: {msg_type.__name__})")

    def listener_callback(self, msg, topic):
        try:
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                sec = msg.header.stamp.sec
                nanosec = msg.header.stamp.nanosec
                timestamp = f"{sec}.{nanosec:09d}"
                self.get_logger().info(f"Topic '{topic}': Message timestamp: {timestamp}")
            else:
                self.get_logger().warn(f"Topic '{topic}': No header with stamp found in message")
        except Exception as e:
            self.get_logger().error(f"Error processing message from '{topic}': {e}")

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(
        description='Subscribe to multiple ROS2 topics and print message timestamps.')
    parser.add_argument('topics', nargs='+', help='List of topic names to subscribe to.')
    parsed_args = parser.parse_args()

    node = MultiSubscriber(parsed_args.topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            node.get_logger().error(f"Error during node destruction: {e}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
