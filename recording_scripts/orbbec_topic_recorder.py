import argparse
import subprocess
from datetime import datetime
import time
import signal
import os
import sys
import logging

class Ros2BagRecorder:
    def __init__(self, topic_names, bag_filename=None):
        self.topic_names = topic_names
        self.bag_filename = bag_filename
        self.process = None
        self.bag_directory = None

        # Set up logging
        self.log_folder = 'recording_logs'
        os.makedirs(self.log_folder, exist_ok=True)

        log_filename = os.path.join(self.log_folder, f"orbbec_record_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
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
        if not self.topic_names:
            self.logger.error("No topics provided to record.")
            return
        
        self.bag_directory = os.path.abspath(self.bag_filename)
        command = ["ros2", "bag", "record", "-o", self.bag_filename] + self.topic_names

        print(f"Running command: {' '.join(command)}")
        try:
            self.process = subprocess.Popen(command)
            self.logger.info(f"Recording started. Bag file will be saved in {self.bag_filename}.")
        except Exception as e:
            self.logger.error(f"Error starting ros2 bag record: {e}")

    def stop_recording(self):
        if self.process and self.process.poll() is None:
            self.logger.info("Stopping the recording process...")
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
                self.logger.info("Recording stopped safely.")
                yaml_file = os.path.join(self.bag_directory, "metadata.yaml")
                if os.path.exists(yaml_file):
                    self.logger.info(f"Metadata YAML file created: {yaml_file}")
                else:
                    self.logger.warning("Warning: YAML metadata file was not created.")
            except subprocess.TimeoutExpired:
                self.logger.error("Timeout expired while waiting for the process to stop. Forcing termination.")
                self.process.kill()
        else:
            self.logger.warning("The recording process is not running.")

    def record_for_duration(self, duration=10):
        self.start_recording()
        time.sleep(duration)
        self.stop_recording()

def handle_shutdown(signal_received, frame):
    print("\nReceived shutdown signal. Stopping the ros2 bag recording.")
    recorder.stop_recording()
    sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS2 Bag Recorder with Time-based File Splitting")
    parser.add_argument(
        '--topics', '-t', nargs='+', default=None,
        help="List of ROS2 topics to record. If not provided, default camera topics will be used."
    )
    parser.add_argument(
        '--output', '-o', type=str, default=None,
        help="Base output directory for bag files"
    )
    parser.add_argument(
        '--duration', '-d', type=int, default=300,
        help="Duration (in seconds) for each recording segment"
    )

    args = parser.parse_args()

    # Set default topics
    if args.topics is None:
        args.topics = ["/pen1_camera_1/color/image_raw/compressed", "/pen1_camera_1/depth/image_raw"]

    # Handle Ctrl+C
    signal.signal(signal.SIGINT, handle_shutdown)

    # Use default topics if not provided
    if args.topics is None:
        args.topics = ["/camera/color/image_raw", "/camera/depth/image_raw"]

    # # Set default output dir if not provided
    if args.output is None:
        args.output = "bags"
    os.makedirs(args.output, exist_ok=True)  # <- Ensure base output dir exists!

    # Initialize recorder
    recorder = Ros2BagRecorder(args.topics, args.output)

    while True:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        recorder.start_recording(timestamp)
        time.sleep(args.duration)
        recorder.stop_recording()
