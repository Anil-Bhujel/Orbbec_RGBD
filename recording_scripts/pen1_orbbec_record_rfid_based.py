import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import subprocess
from datetime import datetime, timedelta
import signal
import os
import argparse
import time
import threading
import psutil  # For managing subprocesses and ensuring they stay alive
import logging
from logging.handlers import RotatingFileHandler

class RFIDBasedRecordingNode(Node):
    def __init__(self, topic_names, message_types, max_duration_s, base_bag_dir="/workspaces/isaac_ros-dev/bags/pen1/3D", log_dir="/workspaces/isaac_ros-dev/recording_logs/pen1/3D"):
        super().__init__('camera_recording_node')

        # Parameters
        self.topic_names = topic_names
        self.max_duration_s = max_duration_s
        self.base_bag_dir = base_bag_dir
        self.log_dir = log_dir

        self.rfid_timeout = timedelta(seconds=30)  # 30 seconds timeout after last RFID data
        self.rfid_threshold = timedelta(minutes=5)  # 5 minutes threshold for splitting the bag file
        self.recording_process = None
        self.last_rfid_time = None
        self.recording_start_time = None
        self.is_recording = False
        self.last_split_time = None  # Track the last time we split the recording
        self.last_logged_rfid_time = None  # Track the last logged RFID event time

        # Initialize the logger
        self.logger = self.setup_logger()

        # Subscribers to the topics
        self.create_subscription(String, '/pen1_rfid', self.rfid_callback, 10)

        # Timer to check for RFID timeout and periodic recording
        self.create_timer(1.0, self.check_rfid_timeout)

    def setup_logger(self):
        """Set up logger to log both to console and to a file."""
        logger = logging.getLogger('CameraRecordingLogger')
        logger.setLevel(logging.INFO)

        # Log to console
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

        # Log to file with rotation
        log_folder = self.create_log_folder()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')  # Timestamp for log file (same as bag file)
        log_file = os.path.join(log_folder, f"{timestamp}.log")
        
        os.makedirs(log_folder, exist_ok=True)  # Ensure the log directory exists
        file_handler = RotatingFileHandler(log_file, maxBytes=10 * 1024 * 1024, backupCount=5)  # 10MB per file
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        return logger

    def create_log_folder(self):
        """Create a folder for today's logs based on current date (YYYYMMDD)."""
        today_date = datetime.now().strftime('%Y%m%d')
        log_folder = os.path.join(self.log_dir, today_date)
        os.makedirs(log_folder, exist_ok=True)
        return log_folder

    def rfid_callback(self, msg):
        """Triggered when RFID data is received."""
        current_time = datetime.now()

        # Log the RFID received event only once before starting recording
        if self.last_logged_rfid_time is None or (current_time - self.last_logged_rfid_time > timedelta(seconds=30)):
            self.logger.info("Received RFID data")
            self.last_logged_rfid_time = current_time

        self.last_rfid_time = current_time
        
        if not self.is_recording:  # Start recording only if not already recording
            threading.Thread(target=self.start_recording).start()  # Start recording in a new thread

    def check_rfid_timeout(self):
        """Check if the RFID data timeout has been reached and manage recording."""
        if self.last_rfid_time:
            time_since_last_rfid = datetime.now() - self.last_rfid_time
            if time_since_last_rfid > self.rfid_timeout:  # If timeout exceeds, stop recording
                if self.is_recording:
                    self.stop_recording()
                    self.logger.info("RFID data unavailable for 30 seconds. Stopping recording.")

        if self.is_recording:
            time_since_last_split = datetime.now() - self.last_split_time if self.last_split_time else timedelta(minutes=0)
            if time_since_last_split >= self.rfid_threshold:  # Split bag every 5 minutes
                self.logger.info("5 minutes of RFID data received. Splitting bag.")
                self.stop_recording()  # Stop the current recording
                threading.Thread(target=self.start_recording).start()  # Start a new recording

    def get_new_bag_folder(self):
        """Create a new folder for recording with a timestamp."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        folder_name = os.path.join(self.base_bag_dir, timestamp)
        # os.makedirs(folder_name, exist_ok=True)  # Create folder if it doesn't exist
        return folder_name

    def start_recording(self):
        """Start recording when RFID data is available."""
        self.is_recording = True
        self.logger.info("Starting recording with ros2 bag")
        
        # Create a new folder to store the bag file
        new_bag_folder = self.get_new_bag_folder()

        # Create ros2 bag command with dynamic topics
        topics_str = " ".join(self.topic_names)
        
        # Start the subprocess and run in the background
        try:
            # Log the actual command being run for debugging
            self.logger.info(f"Running ros2 bag with topics: {topics_str} - Output folder: {new_bag_folder}")
            
            self.recording_process = subprocess.Popen(
                ['ros2', 'bag', 'record', *self.topic_names, '-o', new_bag_folder],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )

            # Capture output and errors from the ros2 bag process
            stdout, stderr = self.recording_process.communicate()
            if stdout:
                self.logger.info(f"ros2 bag stdout: {stdout.decode()}")
            if stderr:
                self.logger.error(f"ros2 bag stderr: {stderr.decode()}")

            # Wait for the recording to start properly (give it time to initialize)
            # time.sleep(5)  # Sleep for a bit longer to ensure the process is fully initialized

            self.recording_start_time = datetime.now()  # Mark the time when recording starts
            self.logger.info("Recording started successfully.")
            self.last_split_time = self.recording_start_time  # Set the first split time
            self.ensure_process_is_alive()

            # Let it run for at least the specified max_duration_s (default 60 seconds)
            time.sleep(self.max_duration_s)

        except Exception as e:
            self.logger.error(f"Failed to start ros2 bag process: {e}")
            self.is_recording = False

    def ensure_process_is_alive(self):
        """Ensures the ros2 bag process is alive and running."""
        if self.recording_process:
            try:
                process = psutil.Process(self.recording_process.pid)
                if not process.is_running():
                    self.logger.warning("ros2 bag process has stopped unexpectedly.")
                    self.stop_recording()  # Stop if the process is no longer running
            except psutil.NoSuchProcess:
                self.logger.warning("ros2 bag process not found.")
                self.stop_recording()

    def stop_recording(self):
        """Stop the recording gracefully."""
        if self.recording_process:
            self.logger.info("Stopping recording")
            self.recording_process.send_signal(signal.SIGTERM)  # Gracefully stop ros2 bag (SIGTERM instead of SIGINT)
            self.recording_process.wait()  # Wait for the process to terminate
            self.recording_process = None
            self.recording_start_time = None
        else:
            self.logger.warning("No active recording process to stop.")
        
        self.is_recording = False  # Mark as not recording

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="ROS2 Camera Recorder with RFID control")

    parser.add_argument(
        '--topics', nargs='*', 
        default=[], 
        help="List of topics to subscribe to, separated by space. For default topics, use '--default'."
    )

    parser.add_argument(
        '--types', nargs='*', 
        default=[], 
        choices=['String', 'Image', 'CompressedImage'],
        help="Specify message types for each topic, 'string', 'image', or 'compressed_image'."
    )

    parser.add_argument(
        '--max_duration', type=int, default=60,
        help="Maximum duration in seconds for each ros2 bag recording (default is 60 seconds)."
    )

    parser.add_argument(
        '--bag_directory', type=str, default="/workspaces/isaac_ros-dev/bags/pen1/3D",
        help="Directory for storing ros2 bag files."
    )

    parser.add_argument(
        '--log_directory', type=str, default="/workspaces/isaac_ros-dev/recording_logs/pen1/3D",
        help="Directory for log recording (default is /workspaces/isaac_ros-dev/recording_logs/pen1/3D)"
    )

    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)
    parsed_args = parse_args()

    # Default topics and message types
    default_topics = [
        '/pen1_camera_1/color/image_raw/compressed', '/pen1_camera_1/depth/image_raw',
        '/pen1_camera_2/color/image_raw/compressed', '/pen1_camera_2/depth/image_raw',
        '/pen1_camera_1/color/camera_info', '/pen1_camera_1/depth/camera_info',
        '/pen1_camera_2/color/camera_info', '/pen1_camera_2/depth/camera_info',
        '/pen1_rfid'
    ]
    default_message_types = [CompressedImage, Image, CompressedImage, Image, CameraInfo, CameraInfo, CameraInfo, CameraInfo, String]

    # Use default topics and types if no arguments provided
    if '--default' in parsed_args.topics or not parsed_args.topics:
        topics = default_topics
        types = default_message_types
    else:
        topics = parsed_args.topics
        types = parsed_args.types

    # Create and spin the RFIDBasedRecordingNode
    node = RFIDBasedRecordingNode(
        topic_names=topics,
        message_types=types,
        max_duration_s=parsed_args.max_duration,
        base_bag_dir=parsed_args.bag_directory,
        log_dir=parsed_args.log_directory
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
