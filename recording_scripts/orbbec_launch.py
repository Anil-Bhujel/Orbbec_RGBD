import subprocess
import sys
import time
import signal
import os

def run_ros2_launch(frame_rate):
    container_name = "isaac_ros_dev-x86_64-container"
    
    # Check if the container is running
    print(f"Checking if Docker container {container_name} is running...")
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={container_name}", "--format", "{{.Names}}"],
            check=True, capture_output=True, text=True
        )
        
        if result.stdout.strip() != container_name:
            print(f"Error: Container {container_name} is not running. Starting it...")
            subprocess.run(["docker", "start", container_name], check=True)  # Start container if not running
    except subprocess.CalledProcessError:
        print(f"Error: Could not check Docker container status.")
        return

    # Attach to the running Docker container and execute commands inside it
    print(f"Attaching to running Docker container: {container_name}")
    
    launch_command = f"source /workspaces/isaac_ros-dev/install/setup.bash && ros2 launch orbbec_camera femto_net_camera.launch.py color_fps:={frame_rate} depth_fps:={frame_rate} ir_fps:={frame_rate}"
    
    # Use `docker exec` to run the command inside the container
    try:
        subprocess.run(
            ["docker", "exec", "-it", container_name, "bash", "-c", launch_command],
            check=True
        )
        print(f"ROS 2 launch command executed successfully: {launch_command}")
    
    except subprocess.CalledProcessError as e:
        print(f"Error while executing command inside Docker: {e}")
        return

def stop_previous_launch():
    print("Sending Ctrl+C to stop the previous launch...")
    # Send SIGINT (Ctrl+C) to the process running the ROS 2 launch command
    try:
        subprocess.run(["docker", "exec", "-it", "isaac_ros_dev-x86_64-container", "pkill", "-SIGINT", "ros2"], check=True)
        time.sleep(5)
    except subprocess.CalledProcessError as e:
        print(f"Error stopping previous launch: {e}")

def get_frame_rate_based_on_time():
    current_time = time.strftime("%H:%M:%S", time.localtime()) # Getting current time
    
    ha_start = "05:00:00"
    ha_end = "15:30:00"
    ma_start = "15:30:00"
    ma_end = "18:30:00"
    la_start = "18:30:00"
    la_end = "05:00:00"
    
    if la_end < current_time <= ha_start:
        return 5
    elif ha_start <= current_time < ha_end:
        return 30
    elif ma_start <= current_time < ma_end:
        return 15
    elif la_start <= current_time:
        return 5

def main():
    prev_frame_rate = 0
    while True:
        frame_rate = get_frame_rate_based_on_time()
        print(f"Current frame rate: {frame_rate} FPS")
        
        if prev_frame_rate != frame_rate:
            # If frame rate has changed, stop the previous launch and start a new one
            stop_previous_launch()
            run_ros2_launch(frame_rate)
            prev_frame_rate = frame_rate

        # Wait for 1 minute before checking again
        time.sleep(60)

if __name__ == "__main__":
    main()
