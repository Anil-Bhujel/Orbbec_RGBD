import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import subprocess
import argparse
import os
import time
import threading
from rclpy.qos import QoSProfile
from datetime import datetime
import logging

class SceneBasedRepublisherAndRecorder(Node):
    def __init__(self, topic_names, message_types, rate_hz, max_duration_s, base_bag_dir="/workspaces/isaac_ros-dev/bags/pen2/3D", log_dir="/workspaces/isaac_ros-dev/recording_logs/pen2/3D"):
        super().__init__('scene_based_republisher_and_recorder')

        self.topic_names = topic_names
        self.message_types = message_types
        self.rate_hz = rate_hz
        self.max_duration_s = max_duration_s
        self.base_bag_dir = base_bag_dir
        self.log_dir = log_dir

        self.subscriptions_dict = {}
        self.publishers_dict = {}

        self.last_processed_time = {topic: None for topic in self.topic_names}
        self.messages_dict = {topic: None for topic in self.topic_names}

        self.bag_process = None
        self.lock = threading.Lock()

        # Initialize logger for log file
        self.log_file_path = None
        self.log_file = None
        self.setup_logging()

        # Timer for periodic checks and message republishing
        self.timer = None
        self.update_timer()

        # Thread for handling bag recording
        self.recording_thread = threading.Thread(target=self.start_bag_recording, daemon=True)
        self.recording_thread.start()

        # Scene checking logic (simplified)
        self.scene_type = "medium_activity"
        self.scene_checking_thread = threading.Thread(target=self.check_scene, daemon=True)
        self.scene_checking_thread.start()

        # Create subscriptions for the provided topics
        self.check_available_topics()

    def get_new_bag_folder(self):
        """Create a new folder for recording with a timestamp."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        folder_name = f"{self.base_bag_dir}/{timestamp}"
        return folder_name

    def get_log_file_path(self, log_folder):
        """Get the path for the log file corresponding to the current bag recording."""
        log_folder = log_folder + "/" + datetime.now().strftime('%Y%m%d')  # Day-wise sub-folder
        os.makedirs(log_folder, exist_ok=True)
        log_file_name = datetime.now().strftime('%Y%m%d_%H%M%S') + ".log"
        log_file_path = os.path.join(log_folder, log_file_name)
        return log_file_path
    
    def setup_logging(self):
        """Sets up the logging configuration."""
        log_folder = self.get_log_file_path(self.log_dir)  # Make sure this is created properly
        os.makedirs(os.path.dirname(log_folder), exist_ok=True)
        self.log_file_path = log_folder
        
        logging.basicConfig(filename=log_folder, level=logging.DEBUG)
        self.get_logger().info(f"Logging to {log_folder}")

    def start_bag_recording(self):
        """Start ros2 bag recording process."""
        while rclpy.ok():
            with self.lock:
                if not self.bag_process:
                    self.current_bag_folder = self.get_new_bag_folder()
                    bag_file = self.current_bag_folder
                    topics_list = [f"/pen2{topic}" for topic in self.topic_names]

                    # Adding camera info of pen1
                    camera_infos = ["/pen2_rfid", "/camera_03/color/camera_info", "/camera_03/depth/camera_info", "/camera_04/color/camera_info", "/camera_04/depth/camera_info"]
                    topics_list.extend(camera_infos)

                    # Open log file before starting bag process
                    self.setup_logging()
                    self.log_file = open(self.log_file_path, 'w')
                    
                    # Start the ros2 bag process and redirect output to log file
                    self.bag_process = subprocess.Popen(
                        ['ros2', 'bag', 'record', '-o', bag_file] + topics_list,
                        stdout=self.log_file,
                        stderr=self.log_file
                    )
                    
                    self.get_logger().info(f"Started recording to {self.current_bag_folder}")
                    logging.info(f"Started recording to {self.current_bag_folder}")

            # Sleep for the max duration or adjust based on scene change
            time.sleep(self.max_duration_s)
            self.stop_bag_recording()

    def stop_bag_recording(self):
        """Stop the ros2 bag recording process."""
        with self.lock:
            if self.bag_process:
                self.get_logger().info(f"Stopping recording in {self.current_bag_folder}")
                logging.info(f"Stopping recording in {self.current_bag_folder}")
                self.bag_process.terminate()
                self.bag_process.wait()
                self.bag_process = None
                
                if self.log_file:
                    self.log_file.close()  # Close the log file properly
            else:
                self.get_logger().info("No active ros2 bag process to stop.")
                logging.info("No active ros2 bag process to stop.")

    def check_scene(self):
        """Simulate dynamic scene checking to adjust the republish rate."""
        while rclpy.ok():
            # Schedule-based dynamic rate
            ha_start = "05:00:00"
            ha_end = "13:00:00"
            ma_start = "13:00:00"
            ma_end = "15:00:00"
            la_start = "15:00:00"
            la_end = "05:00:00"
            current_time = time.strftime("%H:%M:%S", time.localtime())  # Getting current time

            # Low activity time
            if la_end < current_time <= ha_start:
                self.scene_type = "low_activity"
                self.rate_hz = 5
            elif ha_start <= current_time < ha_end:
                self.scene_type = "high_activity"
                self.rate_hz = 30
            elif ma_start <= current_time < ma_end:
                self.scene_type = "medium_activity"
                self.rate_hz = 15
            elif la_start <= current_time:
                self.scene_type = "low_activity"
                self.rate_hz = 5

            # Update republish rate dynamically
            self.get_logger().debug(f"Scene type: {self.scene_type}, Adjusted rate: {self.rate_hz} Hz")

            # Update the timer rate dynamically
            self.update_timer()

            # Sleep for some time before checking again
            time.sleep(3600)

    def update_timer(self):
        """Dynamically update the rate of the timer."""
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()  # Cancel the old timer
            self.get_logger().info(f"Canceled old timer due to rate change to {self.rate_hz} Hz")

        # Create a new timer with the updated rate
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)
        self.get_logger().info(f"New timer created with rate: {self.rate_hz} Hz")

    def check_available_topics(self):
        """Check the availability of the provided topics and handle subscriptions."""
        available_topics = [topic[0] for topic in self.get_topic_names_and_types()]
        self.get_logger().debug(f"Available topics: {available_topics}")

        # Subscribe to the topics provided in the argument
        for topic, msg_type in zip(self.topic_names, self.message_types):
            if topic in available_topics:
                if topic not in self.subscriptions_dict:
                    qos_profile = QoSProfile(depth=10)
                    self.subscriptions_dict[topic] = self.create_subscription(
                        msg_type,
                        topic,
                        self.create_listener_callback(topic),
                        qos_profile
                    )
                    self.get_logger().debug(f"Subscribed to {topic}")
            else:
                if topic in self.subscriptions_dict:
                    self.get_logger().info(f"Topic {topic} is unavailable, unsubscribing.")
                    self.subscriptions_dict[topic].destroy()
                    self.subscriptions_dict.pop(topic)

    def create_listener_callback(self, topic):
        """Create a listener callback function for each topic."""
        def listener_callback(msg):
            self.messages_dict[topic] = msg
            self.last_processed_time[topic] = self.get_clock().now()
            self.get_logger().debug(f"Received message on topic {topic}")
        return listener_callback

    def timer_callback(self):
        """Periodically check and handle message republishing."""
        for topic, msg in self.messages_dict.items():
            if msg:
                self.republish_message(topic, msg)

    def republish_message(self, topic, msg):
        """Republish the message on a new topic."""
        new_topic = f"/pen2{topic}"

        # Create a publisher for the new topic if it doesn't exist
        if new_topic not in self.publishers_dict:
            msg_type = type(msg)
            self.publishers_dict[new_topic] = self.create_publisher(msg_type, new_topic, QoSProfile(depth=10))

        # Republish the message
        self.get_logger().debug(f"Republishing message to {new_topic}")
        self.publishers_dict[new_topic].publish(msg)

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="ROS2 Scene-based Republisher and Recorder")

    parser.add_argument(
        '--topics', nargs='*', 
        default=[], 
        help="List of topics to subscribe to, separated by space. For default topics, use '--default'."
    )

    parser.add_argument(
        '--types', nargs='*', 
        default=[],
        choices=['string', 'image', 'compressed_image'],
        help="Specify message types for each topic, 'string', 'image', or 'compressed_image'."
    )

    parser.add_argument(
        '--rate', type=int, default=30, 
        help="Initial rate in Hz for processing messages (default is 30 Hz)."
    )

    parser.add_argument(
        '--max_duration', type=int, default=300,
        help="Maximum duration in seconds for each ros2 bag recording (default is 300 seconds)."
    )

    parser.add_argument(
        '--bag_directory', type=str, default="/workspaces/isaac_ros-dev/bags/pen2/3D",
        help="Directory for storing the bag file (default is /workspaces/isaac_ros-dev/bags/pen2/3D)."
    )

    parser.add_argument(
        '--log_directory', type=str, default="/workspaces/isaac_ros-dev/recording_logs/pen2/3D",
        help="Directory for log recording (default is /workspaces/isaac_ros-dev/recording_logs/pen2/3D)"
    )

    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)

    parsed_args = parse_args()

    # Set default topics and message types
    default_topics = [
        '/pen2_camera_1/color/image_raw/compressed', '/pen2_camera_1/depth/image_raw',
        '/pen2_camera_2/color/image_raw/compressed', '/pen2_camera_2/depth/image_raw'
    ]
    default_message_types = [CompressedImage, Image, CompressedImage, Image]

    if '--default' in parsed_args.topics or not parsed_args.topics:
        topics = default_topics
        types = default_message_types
    else:
        topics = parsed_args.topics
        types = []

        type_mapping = {
            'string': String,
            'image': Image,
            'compressed_image': CompressedImage,
        }

        for t in parsed_args.types:
            types.append(type_mapping[t])

    # Create the subscriber and start the node
    scene_based_republisher_and_recorder = SceneBasedRepublisherAndRecorder(
        topics, types, parsed_args.rate, parsed_args.max_duration, parsed_args.bag_directory, parsed_args.log_directory
    )

    try:
        rclpy.spin(scene_based_republisher_and_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
