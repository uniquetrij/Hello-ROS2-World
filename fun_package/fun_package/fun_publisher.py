"""
fun_publisher.py

This script defines a ROS 2 publisher node that publishes string messages to the '/fun_topic' topic.
The node increments a counter and includes it in the message being published every second.

Author: Trijeet Modak
Email: uniquetrij@gmail.com
License: Apache-2.0
"""

import rclpy # ROS client library for python
from rclpy.node import Node # # Base class for creating nodes
from std_msgs.msg import String # For publishing string messages

class FunPublisher(Node):
    """
    A ROS 2 Node that publishes messages to the '/fun_topic' topic.
    """

    def __init__(self):
        """
        Initializes the FunPublisher node.
        - Creates a publisher for the '/fun_topic' topic.
        - Sets up a timer to publish messages every second.
        """
        super().__init__('fun_publisher')
        self.publisher_ = self.create_publisher(String, '/fun_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second
        self.get_logger().info('FunPublisher is now publishing to /fun_topic')
        self.counter = 0  # Counter to track the number of messages published

    def publish_message(self):
        """
        Publishes a message to the '/fun_topic' topic.
        - Increments the counter and includes it in the message.
        """
        self.counter += 1
        msg = String()
        msg.data = f'Hello {self.counter} from FunPublisher!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    """
    Main function to initialize and run the FunPublisher node.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    fun_publisher = FunPublisher()  # Create an instance of the FunPublisher node
    try:
        rclpy.spin(fun_publisher)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully on Ctrl+C
    finally:
        fun_publisher.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown the ROS 2 Python client library

if __name__ == '__main__':
    main()
    
