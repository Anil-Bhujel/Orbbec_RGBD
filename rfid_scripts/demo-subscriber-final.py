import tkinter as tk
from tkinter import Label
import msu_rfid_reader_publisher_v2 as reader
from pubsub import pub
from datetime import datetime as dt
import threading
import json

# Import ROS2 modules
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException


class DemoApp:
    def __init__(self, master, ros_node):
        self.master = master
        self.ros_node = ros_node
        master.title("DEMO APP - Subscribes to Topic published by RFIDReaderApp")

        self.label = Label(master, text="Waiting for RFID data...")
        self.label.pack(pady=100)

        # Subscribe to the pubsub topic (named topic_rfid)
        self.subscribe_to_topic()

    def subscribe_to_topic(self):
        pub.subscribe(self.receive_message, "topic_rfid")

    def receive_message(self, data):
        # Update the label when a message is received
        self.label.config(
            text=f"Message Received at {dt.strftime(dt.now(), '%Y-%m-%d %H:%M:%S')}, Content = {data}"
        )
        # Publish the received message through the ROS2 publisher node
        self.ros_node.publish_rfid_data(data)


# ROS2 Node that publishes RFID data to two different topics based on the message content.
class RFIDROS2Publisher(Node):
    def __init__(self):
        super().__init__('rfid_ros2_publisher')
        # Create two publishers: one for pen1 and one for pen2.
        self.pen1_pub = self.create_publisher(String, 'pen1_rfid', 10)
        self.pen2_pub = self.create_publisher(String, 'pen2_rfid', 10)

    def publish_rfid_data(self, data):
        # If the data is empty, log and return.
        if not data:
            self.get_logger().info("No RFID data available, waiting for next message.")
            return

        # If data is a dictionary, filter reads by pen value.
        if isinstance(data, dict) and "reads" in data:
            time_of_read = data.get("time_of_read", dt.strftime(dt.now(), "%Y-%m-%d %H:%M:%S"))
            reads = data["reads"]
            # Filter reads for pen1 and pen2
            pen1_reads = [read for read in reads if read.get("pen") == 1]
            pen2_reads = [read for read in reads if read.get("pen") == 2]

            if pen1_reads:
                msg_data = json.dumps({"time_of_read": time_of_read, "reads": pen1_reads})
                new_msg = String()
                new_msg.data = msg_data
                self.pen1_pub.publish(new_msg)
                self.get_logger().info("Republished RFID data on /pen1_rfid topic.")
            else:
                self.get_logger().warning("No pen1 data in RFID message.")

            if pen2_reads:
                msg_data = json.dumps({"time_of_read": time_of_read, "reads": pen2_reads})
                new_msg = String()
                new_msg.data = msg_data
                self.pen2_pub.publish(new_msg)
                self.get_logger().info("Republished RFID data on /pen2_rfid topic.")
            else:
                self.get_logger().warning("No pen2 data in RFID message.")

        # If data is a string, use simple keyword checking.
        elif isinstance(data, str):
            new_msg = String()
            new_msg.data = data
            data_lower = data.lower()
            if "pen1" in data_lower:
                self.pen1_pub.publish(new_msg)
                self.get_logger().info("Republished RFID data on /pen1_rfid topic.")
            elif "pen2" in data_lower:
                self.pen2_pub.publish(new_msg)
                self.get_logger().info("Republished RFID data on /pen2_rfid topic.")
            else:
                self.get_logger().warning("RFID data does not specify pen1 or pen2. Not republishing.")
        else:
            self.get_logger().warning("Received data in unexpected format. Not republishing.")


def ros2_spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        node.get_logger().info("ROS2 spin shutdown externally.")
    except KeyboardInterrupt:
        node.get_logger().info("ROS2 spin interrupted by KeyboardInterrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    # Initialize ROS2 before creating any nodes.
    rclpy.init()

    # Create the ROS2 publisher node.
    ros_node = RFIDROS2Publisher()

    # Start the ROS2 node in a separate thread.
    ros_thread = threading.Thread(target=ros2_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # This main GUI owns the root window.
    winMain = tk.Tk()
    winMain.geometry('1200x400')
    mainApp = DemoApp(winMain, ros_node)

    # The second GUI is the RFID HUB.
    width = 1270
    height = 360
    win2 = tk.Toplevel(winMain)
    win2.title("RFID Data Acquisition System - UNL+MSU")
    win2.geometry(f"{width}x{height}")  # Keep the width and height of the RFID App
    reader_app = reader.RFIDReaderApp(win2)

    # Run the Tkinter main loop.
    winMain.mainloop()
