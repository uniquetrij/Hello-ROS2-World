"""
fun_subscriber.py

This script defines a ROS 2 subscriber node that listens to string messages on the '/fun_topic' topic.

Author: trijeetkr.modak
Email: uniquetrij@gmail.com
License: Apache-2.0
"""

import rclpy  # ROS client library for Python
from rclpy.node import Node  # Base class for creating nodes
from std_msgs.msg import String  # For subscribing to string messages


class FunSubscriber(Node):
    """
    A ROS 2 Node that subscribes to the '/fun_topic' topic.
    """

    def __init__(self):
        """
        Initializes the FunSubscriber node.
        - Creates a subscription to the '/fun_topic' topic.
        """
        super().__init__('fun_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/fun_topic',
            self.listener_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('FunSubscriber is now listening to /fun_topic')

    def listener_callback(self, msg):
        """
        Callback function that gets executed when a message is received.
        - Logs the received message.
        """
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the FunSubscriber node.
    """
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    fun_subscriber = FunSubscriber()  # Create an instance of the FunSubscriber node
    try:
        rclpy.spin(fun_subscriber)  # Keep the node running
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully on Ctrl+C
    finally:
        fun_subscriber.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == '__main__':
    main()
