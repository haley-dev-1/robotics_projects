#!/usr/bin/env python3
"""
Student main program
--------------------
Replace the contents of _SpiralNode._spiral_callback with your own logic.
"""
import os
import rclpy

from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from drive_spiral.button_node import ButtonNode


class SpiralNode(Node):
    """
    You should finish this node to spiral your iRobot Create 3 in a CCW or CW spiral
    with growth determined by command line parameter or default.
    """
    def __init__(self, demo_mode=False):
        super().__init__('spiral_node')

        # Declare parameter with default value, fetch value, log result
        self.declare_parameter('spiral_scale', 1.0)
        self.spiral_scale = self.get_parameter('spiral_scale').value
        self.get_logger().info(f"Spiral scale set to {self.spiral_scale}")

        # Create publisher to drive robot when button presses are detected
        self.cmd_vel_pub = self.create_publisher(
                Twist,      # Publishes Twist messages
                'cmd_vel',  # Topic to which its subscribers subscribe
                10)         # message buffer (for network congestion)
        self.timer = self.create_timer(
                0.1,                    # Run at 10Hz
                self._publish_twist)    # Callback to link to publisher above
        self.current_twist = Twist()    # Value sent on publication

        # Subscribe to the ButtonNode's spiral_cmd
        self.sub = self.create_subscription(
            String,                 # Expects String messages
            'proj1/spiral_cmd',     # Topic to which it subscribes
            self._spiral_callback,  # Callback to update motion strategy
            10)                     # Message

        # You will need class members for a motion strategy; you might put them here


        # Book-keeping
        self.demo_mode = demo_mode  # useful when on robot
        self.get_logger().info(f"SpiralNode ready. demo_mode = {self.demo_mode}")


    def _spiral_callback(self, msg: String):
        self.get_logger().info(f"Spiral Command: {msg.data}")
        twist = Twist()  # default constructor zeros other velocities
        twist.linear.x = +0.05  # can only ``surge''
        if msg.data == "CCW":
            twist.angular.z = +0.3  # can only yaw
        elif msg.data == "CW":
            twist.angular.z = -0.3  # can only yaw
        elif msg.data == "HALT":
            twist.linear.x = twist.angular.z = 0.0  # STOP!
        self.current_twist = twist


    def _publish_twist(self):
        self.cmd_vel_pub.publish(self.current_twist)



def main(args=None):
    """
    Emits basic environment information for confirmation. Creates two nodes:
      1.) ButtonNode
      2.) SpiralNode
    """
    # Initialize the ROS 2 client library for Python
    rclpy.init(args=args)

    # iRobot Create config information
    logger = rclpy.logging.get_logger("main")

    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')  # defaults to 0
    logger.info(f"ROS_DOMAIN_ID: {domain_id}")

    namespace = os.environ.get('ROS_NAMESPACE', '')  # empty if unset
    logger.info(f"ROS_NAMESPACE: {namespace}")

    rmw = rclpy.get_rmw_implementation_identifier()  # Should be fastrtps
    logger.info(f"RMW_IMPLEMENTATION: {rmw}")

    # Node executor: needed when spinning up multiple nodes in a single process
    executor = MultiThreadedExecutor()

    # ButtonNode node
    button_node = ButtonNode(demo_mode=False)  # switch True to turn off beeps
    executor.add_node(button_node)

    # SpiralNode node
    spiral_node = SpiralNode()
    executor.add_node(spiral_node)

    # Launch nodes and cleanup on keyboard interrupt
    try:
        executor.spin()
    except KeyboardInterrupt:
        logger.info("Recieved keyboard interrupt. Halting.")
    finally:
        button_node.destroy_node()
        spiral_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
