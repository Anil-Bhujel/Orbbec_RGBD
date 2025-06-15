import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import subprocess
import argparse
from rclpy.qos import QoSProfile
from datetime import datetime
import time

class MultiTopicSubscriber(Node):
    def __init__(self, topic_names, message_types, rate_hz, republish_topics, bag_file="/workspaces/isaac_ros-dev/bags/my_bagfile"):
        super().__init__('multi_topic_subscriber')
        self.topic_names = topic_names
        self.message_types = message_types
        self.rate_hz = rate_hz
        self.republish_topics = republish_topics
        self.bag_file = bag_file

        # Initialize a separate dictionary to store subscriptions
        self.subscriptions_dict = {}
        self.publishers_dict = {}

        # Time control: Track the last processed time for each topic to implement rate limiting
        self.last_processed_time = {topic: time.time() for topic in self.topic_names}

        # Set up a Rate object to control the publishing frequency
        self.publish_rate = 1.0 / self.rate_hz  # Desired rate in seconds

        # Set up a timer to periodically check for available messages
        self.timer = self.create_timer(self.publish_rate, self.timer_callback)

        # Start the first check to handle topics right away
        self.check_available_topics()

        # Start a single ros2 bag process to record all topics
        self.bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', self.bag_file] + self.republish_topics
        )

        # Store the latest received messages
        self.messages_dict = {topic: None for topic in self.topic_names}

    def create_listener_callback(self, topic):
        """Create a listener callback function for each topic"""
        def listener_callback(msg):
            self.messages_dict[topic] = msg
            self.last_processed_time[topic] = time.time()
            self.get_logger().info(f"Received message on {topic}")
        return listener_callback

    def republish_message(self, topic, msg):
        """Republish the message on a new topic"""
        new_topic = f"/pen1{topic}"
        
        # Check if the publisher exists for this new topic
        if new_topic not in self.publishers_dict:
            # Create a publisher for the new topic
            msg_type = type(msg)
            self.publishers_dict[new_topic] = self.create_publisher(msg_type, new_topic, QoSProfile(depth=10))

        # Republish the message
        self.publishers_dict[new_topic].publish(msg)
        self.get_logger().info(f"Republished message to {new_topic}")

    def timer_callback(self):
        """Periodically check and republish the latest messages at the specified rate"""
        for topic, msg in self.messages_dict.items():
            if msg and (time.time() - self.last_processed_time[topic] >= self.publish_rate):
                self.republish_message(topic, msg)

    def check_available_topics(self):
        """Check the availability of the provided topics and handle subscriptions"""
        # Get only the topics that are provided
        available_topics = [topic[0] for topic in self.get_topic_names_and_types()]

        for topic, msg_type in zip(self.topic_names, self.message_types):
            if topic in available_topics:
                # Topic is available, subscribe if not already done
                if topic not in self.subscriptions_dict:
                    qos_profile = QoSProfile(depth=10)
                    self.subscriptions_dict[topic] = self.create_subscription(
                        msg_type,
                        topic,
                        self.create_listener_callback(topic),
                        qos_profile
                    )
                else:
                    self.get_logger().info(f"Already subscribed to {topic}.")
            else:
                # Topic is unavailable, stop subscription if it was started
                if topic in self.subscriptions_dict:
                    self.get_logger().info(f"Topic {topic} is unavailable. Dropping it.")
                    self.subscriptions_dict[topic].destroy()
                    self.subscriptions_dict.pop(topic)

    def stop_recording(self):
        """Stop ros2 bag recording process."""
        self.get_logger().info("Stopping ros2 bag recording")
        self.bag_process.terminate()

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="ROS2 Multi-topic Subscriber with Bag Recording")

    # Option for topics
    parser.add_argument(
        '--topics', nargs='*', 
        default=[], 
        help="List of topics to subscribe to, separated by space. For default topics, use '--default'."
    )

    # Option for message types
    parser.add_argument(
        '--types', nargs='*', 
        default=[],
        choices=['string', 'image', 'compressed_image'],
        help="Specify message types for each topic, 'string', 'image', or 'compressed_image'."
    )

    # Frequency for checking messages (Hz)
    parser.add_argument(
        '--rate', type=int, default=1, 
        help="Rate in Hz for processing messages (e.g., 1, 15, 30). Default is 1 Hz."
    )

    # Option to specify republishing topics
    parser.add_argument(
        '--republish_topics', nargs='*', 
        default=[], 
        help="List of topics to republish at the specified rate, separated by space."
    )

    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)

    # Parse arguments
    parsed_args = parse_args()

    filename = datetime.now().strftime('%Y%m%d_%H%M%S')
    directory = "/workspaces/isaac_ros-dev/bags/"

    recording_filename = directory + filename

    # Set default topics and message types
    default_topics = ['/pen1_rfid','/camera_01/depth/image_raw','/camera_01/color/image_raw/compressed','/camera_01/color/camerainfo',
                      '/camera_02/depth/image_raw','/camera_02/color/image_raw/compressed','/camera_02/color/camerainfo']
    default_message_types = [String, Image, CompressedImage, CameraInfo, Image, CompressedImage, CameraInfo]

    # If no topics are passed, use default ones
    if '--default' in parsed_args.topics or not parsed_args.topics:
        topics = default_topics
        types = default_message_types
    else:
        # Validate that the number of topics and types match
        if len(parsed_args.topics) != len(parsed_args.types):
            raise ValueError("The number of topics must match the number of message types.")
        
        topics = parsed_args.topics
        types = []
        
        # Map the passed types to actual message types
        type_mapping = {
            'string': String,
            'image': Image,
            'compressed_image': CompressedImage,
        }

        for t in parsed_args.types:
            types.append(type_mapping[t])

    # Republish topics to be recorded
    republish_topics = [f'pen2{topic}' for topic in topics]

    # Create the subscriber and start the node
    multi_topic_subscriber = MultiTopicSubscriber(topics, types, parsed_args.rate, republish_topics, recording_filename)

    try:
        # Keep the node running
        rclpy.spin(multi_topic_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Graceful shutdown
        multi_topic_subscriber.stop_recording()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
