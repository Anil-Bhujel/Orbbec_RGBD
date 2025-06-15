import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rclpy.qos import QoSProfile
import time
import threading
from datetime import datetime
import os

class DynamicImageSubscriber(Node):
    def __init__(self, topics, rates, max_duration=60, base_bag_dir="/workspaces/isaac_ros-dev/bags"):
        super().__init__('dynamic_image_subscriber')
        
        self.topics = topics
        self.rates = rates  # Dictionary of topics and their respective rates
        self.max_duration = max_duration
        self.base_bag_dir = base_bag_dir
        self.subscribers = {}
        self.publishers = {}
        self.messages = {}
        self.last_processed_time = {}
        self.lock = threading.Lock()
        
        # Dynamic subscription
        self._subscribe_to_topics()

        # Thread for handling publishing at different rates
        self.timer = self.create_timer(0.1, self.timer_callback)  # Check every 100ms

    def _subscribe_to_topics(self):
        """Create subscribers for the requested topics."""
        qos_profile = QoSProfile(depth=10)
        
        for topic in self.topics:
            # Determine the message type (e.g., Image or CompressedImage)
            msg_type = self._get_message_type_for_topic(topic)
            
            # Create the subscription dynamically
            self.subscribers[topic] = self.create_subscription(
                msg_type,
                topic,
                self.create_listener_callback(topic),
                qos_profile
            )
            self.get_logger().info(f"Subscribed to {topic} at {self.rates.get(topic, 30)} Hz")

    def _get_message_type_for_topic(self, topic):
        """Determine the appropriate message type based on the topic name."""
        if "compressed" in topic:
            return CompressedImage
        elif "image" in topic:
            return Image
        elif "camerainfo" in topic:
            return CameraInfo
        else:
            return String

    def create_listener_callback(self, topic):
        """Create a listener callback for each topic."""
        def listener_callback(msg):
            with self.lock:
                self.messages[topic] = msg
                self.last_processed_time[topic] = self.get_clock().now()
        return listener_callback

    def timer_callback(self):
        """Handle message republishing based on specified rates."""
        current_time = self.get_clock().now()
        
        for topic, msg in self.messages.items():
            if msg:
                rate = self.rates.get(topic, 30)  # Default rate is 30 Hz
                if current_time.seconds_nanoseconds()[0] % (1 / rate) == 0:
                    self.republish_message(topic, msg)

    def republish_message(self, topic, msg):
        """Republish the message at the desired rate."""
        new_topic = f"/pen1{topic}"

        # Create a new publisher for each topic if not already created
        if new_topic not in self.publishers:
            msg_type = type(msg)
            self.publishers[new_topic] = self.create_publisher(msg_type, new_topic, QoSProfile(depth=10))

        self.publishers[new_topic].publish(msg)
        self.get_logger().info(f"Republishing {topic} at {self.rates.get(topic, 30)} Hz")

def main(args=None):
    rclpy.init(args=args)
    
    topics = [
        '/camera_01/color/image_raw',
        '/camera_01/depth/image_raw',
        '/camera_02/color/image_raw',
        '/camera_02/depth/image_raw',
    ]
    
    # Dictionary mapping topics to their respective desired rates (Hz)
    rates = {
        '/camera_01/color/image_raw': 30,   # 30 Hz
        '/camera_01/depth/image_raw': 15,   # 15 Hz
        '/camera_02/color/image_raw': 1,    # 1 Hz
        '/camera_02/depth/image_raw': 15,   # 15 Hz
    }

    # Create and start the subscriber node
    dynamic_subscriber = DynamicImageSubscriber(topics, rates)
    
    try:
        rclpy.spin(dynamic_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
