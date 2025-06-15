import subprocess
from datetime import datetime
import time
import signal
import os
import sys
import logging
import argparse
from rclpy.node import Node
import rclpy
#from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden

class TopicChecker(Node):
    def __init__(self, topic_name):
        super().__init__('topic_checker')
        self.topic_name = topic_name
        self.topic_found = False

    def check_topic(self):
        # Get all available topics
        topics = self.get_topic_names_and_types()
        
        # Check if the desired topic exists
        for topic, types in topics:
            if topic == self.topic_name:
                self.get_logger().info(f"Topic '{self.topic_name}' is available with type(s): {types}")
                self.topic_found = True
                return True
        
        self.get_logger().info(f"Topic '{self.topic_name}' is not available.")
        return False
    
class Ros2BagRecorder:
    def __init__(self, topic_names, bag_filename="/workspaces/isaac_ros-dev/bags/my_bagfile"):
        self.topic_names = topic_names
        self.bag_filename = bag_filename
        self.process = None
        self.bag_directory = None


        # Set up logging
        self.log_folder = '/workspaces/isaac_ros-dev/recording_logs'
        os.makedirs(self.log_folder, exist_ok=True)  # Ensure log folder exists
        
         # Log file path with timestamp
        log_filename = os.path.join(self.log_folder, f"ros2_bag_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_filename),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)

    def start_recording(self):
        """Start the ros2 bag record command as a subprocess."""
        if not self.topic_names:
            # raise ValueError("No topics provided to record.")
            self.logger.error(f"No topics provided to record.")
        
        # Create the output directory if it doesn't exist
        self.bag_directory = os.path.abspath(self.bag_filename)
        # if not os.path.exists(self.bag_directory):
        #     os.makedirs(self.bag_directory)

        # Construct the command to run the ros2 bag record command
        command = ["ros2", "bag", "record", "-o", self.bag_filename] + self.topic_names
        
        print(f"Running command: {' '.join(command)}")
        
        try:
            # Start the subprocess to record the ROS2 bag
            self.process = subprocess.Popen(command)
            # print(f"Recording started. Bag file will be saved in {self.bag_filename}.")
            self.logger.info(f"Recording started. Bag file will be saved in {self.bag_filename}.")
        except Exception as e:
            # print(f"Error starting ros2 bag record: {e}")
            self.logger.error(f"Error starting ros2 bag record: {e}")

    def stop_recording(self):
        """Stop the ros2 bag record process safely."""
        if self.process and self.process.poll() is None:
            # print("Stopping the recording process...")
            self.logger.info("Stopping the recording process...")
            self.process.terminate()  # Send a termination signal to the process
            
            try:
                self.process.wait(timeout=10)  # Wait for the process to terminate
                # print("Recording stopped safely.")
                self.logger.info("Recording stopped safely.")
                
                # Check if the YAML metadata file is created
                yaml_file = os.path.join(self.bag_directory, "metadata.yaml")
                if os.path.exists(yaml_file):
                    # print(f"Metadata YAML file created: {yaml_file}")
                    self.logger.info(f"Metadata YAML file created: {yaml_file}")
                else:
                    # print("Warning: YAML metadata file was not created.")
                    self.logger.warning("Warning: YAML metadata file was not created.")
            except subprocess.TimeoutExpired:
                print("Timeout expired while waiting for the process to stop. Forcing termination.")
                self.process.kill()  # Force kill the process if it doesn't stop within the timeout
        else:
            print("The recording process is not running.")

    def record_for_duration(self, duration=10):
        """Record for a specified duration (in seconds) and then stop."""
        self.start_recording()
        time.sleep(duration)  # Let the recording run for the specified duration
        self.stop_recording()

def handle_shutdown(signal, frame):
    """Gracefully handle Ctrl+C or other shutdown signals."""
    print("\nReceived shutdown signal. Stopping the ros2 bag recording.")
    recorder.stop_recording()
    sys.exit(0)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Orbbec Recording Module")
    parser.add_argument('--compression', type=str, default="MJPG", help='Color compression format')
    parser.add_argument('--duration', type=int, default=300, help='Duration of the recording in seconds')

    return parser.parse_args()

if __name__ == "__main__":
    # rclpy.init(args=args)
    # # Specify the topic to check
    # topic_to_check = '/camera_01/color/h26x_encoded_data'
    # checker = TopicChecker(topic_to_check)
    
    # # Check for the topic
    # h26x_topic = checker.check_topic()
    # # Shutdown the node
    # checker.destroy_node()
    # rclpy.shutdown()

    while(True):
        # Register the signal handler for Ctrl+C (SIGINT)
        signal.signal(signal.SIGINT, handle_shutdown)

        args = parse_arguments()

        # Example usage: Record specific topics
        if(args.compression == "MJPG"):
        #if(not h26x_topic):
            topics_to_record = ["/camera_01/color/image_raw/compressed", "/camera_01/depth/image_raw","/camera_02/color/image_raw/compressed", "/camera_02/depth/image_raw"]
        
        elif(args.compression == "h264" or args.compression == "h265"):
        #else:
            topics_to_record = ["/camera_01/color/h26x_encoded_data", "/camera_01/depth/image_raw","/camera_02/color/h26x_encoded_data", "/camera_02/depth/image_raw"]
        
        #topics_to_record = ["/camera_01/color/image_raw/compressed", "/camera_01/depth/image_raw","/camera_02/color/image_raw/compressed", "/camera_02/depth/image_raw"]
        
        filename = datetime.now().strftime('%Y%m%d_%H%M%S')
        directory = "/workspaces/isaac_ros-dev/bags/pen1/3D/"
        recording_filename = directory+filename
        #recording_duration = 60 # arg.pars
        recording_duration = args.duration
        recorder = Ros2BagRecorder(topics_to_record, recording_filename)

        # Record for 20 seconds (change the duration as needed)
        recorder.record_for_duration(recording_duration)
