#!/usr/bin/env python3
"""Main ROS2 node for Scarface robot fish."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests


class ScarfaceNode(Node):
    """Main node for controlling the Scarface robot fish."""

    def __init__(self):
        super().__init__("scarface_node")
        
        # Declare and get parameters for Arduino connection
        self.declare_parameter("arduino_ip", "192.168.1.17")
        self.declare_parameter("arduino_port", 80)
        self.declare_parameter("request_timeout", 7.0)
        
        self.arduino_ip = self.get_parameter("arduino_ip").value
        self.arduino_port = self.get_parameter("arduino_port").value
        self.request_timeout = self.get_parameter("request_timeout").value
        
        # Map ROS commands to Arduino URL paths
        self.command_map = {
            "U": "U",           # Motor Pulse UP
            "UP": "U",
            "D": "D",           # Motor Pulse DOWN
            "DOWN": "D",
            "L": "L_MOVE",      # Motor Pulse LEFT
            "LEFT": "L_MOVE",
            "R": "R_MOVE",      # Motor Pulse RIGHT
            "RIGHT": "R_MOVE",
            "F": "F",           # Motor Sequence FORWARD
            "FORWARD": "F",
            "S": "S",           # Immediate Halt/Reset
            "STOP": "S"
        }
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, "scarface/status", 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String, "scarface/command", self.command_callback, 10
        )
        
        # Create timer for periodic status updates
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f"Scarface node initialized - Arduino at {self.arduino_ip}:{self.arduino_port}"
        )

    def send_arduino_command(self, command_path):
        """Send an HTTP GET request to the Arduino."""
        url = f"http://{self.arduino_ip}:{self.arduino_port}/{command_path}"
        
        self.get_logger().info(f"Sending command to Arduino: {url}")
        
        try:
            response = requests.get(url, timeout=self.request_timeout)
            
            if response.status_code == 200:
                self.get_logger().info(f"Command successful: {response.text.strip()}")
                
                # Publish status update
                status_msg = String()
                status_msg.data = f"Command '{command_path}' executed: {response.text.strip()}"
                self.publisher_.publish(status_msg)
                
                return True
            else:
                self.get_logger().error(
                    f"Command failed. HTTP Status Code: {response.status_code}"
                )
                return False
                
        except requests.exceptions.Timeout:
            self.get_logger().error(
                f"Timeout: Could not connect to Arduino at {self.arduino_ip}"
            )
            return False
        except requests.exceptions.RequestException as e:
            self.get_logger().error(
                f"Error connecting to Arduino: {e}"
            )
            return False
    
    def command_callback(self, msg):
        """Handle incoming GPS-formatted commands."""
        raw_data = msg.data.strip()
        self.get_logger().info(f'Received raw command: "{raw_data}"')

        try:
            # Example: "1/9/2023-11:05:35.859906,g,-1,-1,-2,255;255"
            # Split by commas and then parse the last part "255;255"
            parts = raw_data.split(",")
            motor_values = parts[-1] if parts else ""
            left_right = motor_values.split(";")

            if len(left_right) == 2:
                left_val = int(left_right[0])
                right_val = int(left_right[1])

                if left_val == 255 and right_val == 255:
                    command_path = self.command_map["FORWARD"]
                elif left_val == 0 and right_val == 255:
                    command_path = self.command_map["LEFT"]
                elif left_val == 255 and right_val == 0:
                    command_path = self.command_map["RIGHT"]
                elif left_val == 0 and right_val == 0:
                    command_path = self.command_map["STOP"]
                else:
                    self.get_logger().warn(
                        f"Unrecognized motor values: {left_val};{right_val}"
                    )
                    return

                self.get_logger().info(
                    f"Parsed as: LEFT={left_val}, RIGHT={right_val} → Command={command_path}"
                )
                self.send_arduino_command(command_path)

            else:
                self.get_logger().warn(f"Invalid command format: '{raw_data}'")

        except Exception as e:
            self.get_logger().error(f"Failed to parse command: {e}")


    def timer_callback(self):
        """Publish periodic status updates."""
        msg = String()
        msg.data = "Scarface operational"
        self.publisher_.publish(msg)
        self.get_logger().debug("Status published")


def main(args=None):
    """Main entry point for the Scarface node."""
    rclpy.init(args=args)
    
    scarface_node = ScarfaceNode()
    
    try:
        rclpy.spin(scarface_node)
    except KeyboardInterrupt:
        pass
    finally:
        scarface_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
