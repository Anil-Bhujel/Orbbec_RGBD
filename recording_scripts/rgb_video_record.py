import os
import time
import subprocess
import logging
import threading
from datetime import datetime


class CameraRecorder:
    def __init__(self, folder_name, camera_topics, log_folder, bag_duration=300):
        self.folder_name = folder_name
        self.log_folder = log_folder
        self.camera_topics = camera_topics
        self.bag_duration = bag_duration
        self.is_recording = False
        self.process = None
        self.current_bag_dir = None
        self.stop_event = threading.Event()  # To signal threads to stop
        self.logger = None  # Initialize logger to None

    def check_topic_availability(self):
        """Check if the topics are available (simulated check)."""
        unavailable_topics = []
        for topic in self.camera_topics:
            # Replace this with actual checks if needed, for example, ROS2 topic checks
            if "unavailable" in topic:  # Simulated topic check
                unavailable_topics.append(topic)
        return unavailable_topics

    def start_recording(self):
        """Starts recording with ros2 bag record in a subprocess."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_filename = f"{self.folder_name}/{timestamp}"
        self.current_bag_dir = bag_filename
        
        # Ensure the log folder exists for both pens
        os.makedirs(self.log_folder, exist_ok=True)
        
        # Create log file with the same name as the bag file
        log_filename = f"{self.log_folder}/{timestamp}_recording.log"
        
        # Set up logging for this specific recording session
        logging.basicConfig(
            filename=log_filename,
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(log_filename)

        # Ensure the directory exists for bag files as well
        os.makedirs(self.folder_name, exist_ok=True)

        # Check topic availability
        unavailable_topics = self.check_topic_availability()
        if unavailable_topics:
            self.logger.warning(f"Some topics are unavailable: {', '.join(unavailable_topics)}")

        # Filter out unavailable topics and only record the available ones
        available_topics = [topic for topic in self.camera_topics if topic not in unavailable_topics]
        if not available_topics:
            self.logger.error("No valid topics to record. Exiting...")
            return  # Exit early if no valid topics to record

        # Command to start recording using ros2 bag record
        cmd = ['ros2', 'bag', 'record', '-o', bag_filename] + available_topics
        self.logger.info(f"Started recording to {bag_filename} with topics {available_topics}...")
        
        try:
            # Start the subprocess to record the topics
            self.process = subprocess.Popen(cmd)
            
            # Wait for the duration of the bag file
            time.sleep(self.bag_duration)
            
            # Stop the current recording and start a new one after the duration
            if not self.stop_event.is_set():
                self.stop_recording()  # Stop current recording

                self.start_recording()  # Restart recording with a new bag file
        except Exception as e:
            self.logger.error(f"Error starting recording for {bag_filename}: {e}")

    def stop_recording(self):
        """Stops the recording subprocess."""
        if self.process:
            self.process.terminate()  # Gracefully terminate the subprocess
            try:
                self.process.wait(timeout=10)  # Wait for 10 seconds for graceful shutdown
                self.logger.info(f"Stopped recording and closed bag file.")
            except subprocess.TimeoutExpired:
                self.logger.warning(f"Recording did not stop in time, forcefully terminating.")
                self.process.kill()  # Forcefully kill if it times out
            # now that recording is done, check for metadata.yaml
            meta_path = os.path.join(self.current_bag_dir, "metadata.yaml")

            if not os.path.exists(meta_path):
                self.logger.warning(f"{meta_path!r} not found, running reindex …")
                try:
                    # ros2 bag reindex is available in Galactic+ :contentReference[oaicite:0]{index=0}
                    subprocess.run(
                        ["ros2", "bag", "reindex", self.current_bag_dir],
                        check=True
                    )
                    self.logger.info("Successfully generated metadata.yaml.")
                except Exception as e:
                    self.logger.error(f"ros2 bag reindex failed: {e}")
            else:
                self.logger.info("metadata.yaml already exists.")

            self.process = None

    def run(self):
        """Runs the recording process continuously in a separate thread."""
        while not self.stop_event.is_set():
            self.start_recording()
            time.sleep(1)  # Add a slight delay to avoid fast looping or race conditions

    def stop(self):
        """Signal to stop the recording and stop the thread."""
        self.stop_event.set()
        self.stop_recording()  # No need to pass logger, it's now a class attribute


def main():
    # Define the folders and camera topics for each pen
    folder_pen1 = "/mnt/data/gilt_data_2025/bags/pen1/2D"  # Pen1 recording path
    folder_pen2 = "/mnt/data/gilt_data_2025/bags/pen2/2D"  # Pen2 recording path

    pen1_log_folder = "/home/pigs/ros2_ws/recording_logs/pen1/2D"  # Pen1 log path
    pen2_log_folder = "/home/pigs/ros2_ws/recording_logs/pen2/2D"  # Pen2 log path

    camera_topics_pen1 = ["/pen1_camera_1/rgb/compressed", "/pen1_camera_2/rgb/compressed", "/pen1_rfid"]
    camera_topics_pen2 = ["/pen2_camera_1/rgb/compressed", "/pen2_camera_2/rgb/compressed", "/pen2_rfid"]

    # Create CameraRecorder instances for each pen and start recording in parallel
    recorder_pen1 = CameraRecorder(folder_pen1, camera_topics_pen1, pen1_log_folder)
    recorder_pen2 = CameraRecorder(folder_pen2, camera_topics_pen2, pen2_log_folder)

    # Run the recording in parallel
    pen1_thread = threading.Thread(target=recorder_pen1.run)
    pen2_thread = threading.Thread(target=recorder_pen2.run)

    # Start both threads to record concurrently
    pen1_thread.start()
    pen2_thread.start()

    # Wait for both threads to complete
    try:
        pen1_thread.join()  # Wait for pen1 thread to complete
        pen2_thread.join()  # Wait for pen2 thread to complete
    except KeyboardInterrupt:
        print("Recording interrupted. Stopping...")
        recorder_pen1.stop()  # Stop the recording for Pen1
        recorder_pen2.stop()  # Stop the recording for Pen2
        
        try:
            pen1_thread.join()  # Ensure pen1 thread is properly stopped
            pen2_thread.join()  # Ensure pen2 thread is properly stopped
        except Exception as e:
            print(f"Error while joining threads: {e}")


if __name__ == "__main__":
    main()