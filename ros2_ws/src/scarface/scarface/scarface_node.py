#!/usr/bin/env python3
"""Main ROS2 node for Scarface robot fish."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ScarfaceNode(Node):
    """Main node for controlling the Scarface robot fish."""

    def __init__(self):
        super().__init__("scarface_node")
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, "scarface/status", 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String, "scarface/command", self.command_callback, 10
        )
        
        # Create timer for periodic status updates
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Scarface node initialized")

    def command_callback(self, msg):
        """Handle incoming commands."""
        self.get_logger().info(f'Received command: "{msg.data}"')
        # Add your command handling logic here

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
