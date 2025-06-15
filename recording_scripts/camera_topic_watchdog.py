import subprocess
import time
import sys

class CameraWatchdog:
    def __init__(self, launch_command, topic_to_monitor, check_interval=15):
        self.launch_command = launch_command
        self.topic_to_monitor = topic_to_monitor
        self.check_interval = check_interval
        self.process = None

    def start_camera_launch(self):
        """Starts the camera launch script."""
        self.process = subprocess.Popen(
            self.launch_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("Camera launch script started...")

    def stop_camera_launch(self):
        """Stops the camera launch script."""
        if self.process:
            print("Stopping camera launch script...")
            self.process.terminate()
            self.process.wait()  # Wait for the process to terminate
            self.process = None
        else:
            print("No running process to stop.")

    def check_camera_topic(self):
        """Checks if the camera topic is available."""
        result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        available_topics = result.stdout.decode('utf-8').splitlines()
        topic_missed = False
        
        for topic in self.topic_to_monitor:
            if topic not in available_topics:
                topic_missed = True
                break
            else:
                topic_missed = False
            
        return topic_missed

    def restart_camera_launch(self):
        """Restarts the camera launch script if the topic is unavailable."""
        print(f"Camera topic {self.topic_to_monitor} is unavailable. Restarting the camera launch...")
        self.stop_camera_launch()
        self.start_camera_launch()

    def monitor(self):
        """Monitors the camera topic and restarts the launch script if necessary."""
        # Start the initial camera launch
        self.start_camera_launch()

        while True:
            time.sleep(self.check_interval)  # Wait for the specified interval
            if self.check_camera_topic():  # If the topic is unavailable
                self.restart_camera_launch()

def main():
    # Configure the watchdog with the launch command and the topic to monitor
    launch_command = ['ros2', 'launch', 'orbbec_camera', 'femto_net_camera.launch.py']
    topic_to_monitor = ['/pen1_camera_1/color/image_raw/compressed','/pen1_camera_1/depth/image_raw','/pen1_camera_2/color/image_raw/compressed',
                        '/pen1_camera_2/depth/image_raw','/pen2_camera_1/color/image_raw/compressed','/pen2_camera_1/depth/image_raw',
                        '/pen2_camera_2/color/image_raw/compressed','/pen2_camera_2/depth/image_raw']  # Modify this as per your camera topic

    watchdog = CameraWatchdog(launch_command, topic_to_monitor)

    try:
        watchdog.monitor()
    except KeyboardInterrupt:
        print("Watchdog interrupted by user.")
        sys.exit(0)

if __name__ == '__main__':
    main()
