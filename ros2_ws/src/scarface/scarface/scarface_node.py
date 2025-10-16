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
        self.declare_parameter("arduino_ip", "192.168.1.25")
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
        """Handle incoming commands."""
        command = msg.data.upper()
        self.get_logger().info(f'Received command: "{command}"')
        
        if command in self.command_map:
            command_path = self.command_map[command]
            self.send_arduino_command(command_path)
        else:
            self.get_logger().warn(
                f"Unknown command '{command}'. Valid commands: {list(self.command_map.keys())}"
            )

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
