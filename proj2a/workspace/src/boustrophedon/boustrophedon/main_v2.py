import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from boustrophedon.button_node import ButtonNode
from boustrophedon.hazard_node import HazardNode
from boustrophedon.odom_node import OdomNode
from boustrophedon.ir_node import IrNode
from geometry_msgs.msg import Twist
import math

class Controller(Node):
    def __init__(self, odom_node, ir_node, pulse_htz=10.0):

        # Updated from original
        super().__init__('Controller')
        self.odom_node = odom_node
        self.ir_node = ir_node

	# Create publisher to drive robot when button presses are detected
        self.cmd_vel_pub = self.create_publisher(
                Twist,      # Publishes Twist messages
                'cmd_vel',  # Topic to which its subscribers subscribe
                10)         # message buffer (for network congestion)

        # Updated from original
        self.dt = 1.0 / float(pulse_htz) # 10hz
        self.create_timer(pulse_htz, self._pulse)
        self.get_logger().info(f"Pulse started, calling pulse() every {pulse_htz}s.")

        '''
        self.timer = self.create_timer(
                0.1,                    # Run at 10Hz
                self._publish_twist)    # Callback to link to publisher above
        '''
        '''
	    String,                 # Expects String messages
            'proj1/spiral_cmd',     # Topic to which it subscribes
            self._spiral_callback,  # Callback to update motion strategy
            10)                     # Message
		'''

        self.spiral = False
        self.spi_dir = 0.0 # start on 0 because spiral is false anyway on init
        self.spi_count = 0
        self.spiral_scale = 1.0
        self.max_count = int(88 * self.spiral_scale)  # optional stop threshold

	# debouncing ("filtering user input before triggering the action")
        self.debounce_s = 0.25
        self.last_button_time = -1e8

        '''
        self.spiral = False
        self.spi_dir = 0
        self.spi_count = 0
		'''

        # Book-keeping
        # self.demo_mode = demo_mode # useful when on robot
        # self.get_logger().info(f"SpiralNode ready. demo_mode = {self.demo_mode}")

    def power_button_handler(self):

        # Updated from original
        self.get_logger().info("Power Button Detected")
        now = self.get_clock().now().nanoseconds *1.e-9
        if now - self.last_button_time < self.debounce_s:
        	return
        self.last_button_time = now

    def hazard_handler(self):
        # Updated from original
        self.get_logger().info("Hazard Detected")
        self.spiral = False # stop spiral active
        self._stop()
        # needs handling... we'll worry about that later, lets just get it to stop if it gets near a wall


    # Updated from original
    def _pulse(self):
        self._handle_odom()
        self._handle_ir()

        if self.spiral:
        	self._do_spiral()
        else:
            self._stop() # stay stopped, we keep this publishing so it doesn't go again

    # spiral motion
    def _do_spiral(self):
        # todo ...
        self.spi_count += 1 # counter for tracking
        v = 0.03 + (self.spi_count * 0.0033)
        w = (0.66 + (0.185 * self.spiral_scale)) * self.spi_dir

        # sanity caps
        v = max(0.03, min(v, 0.30))
        w = max(-1.2, min(w, 1.2))

        self._cmd(v, w)

    def _cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def _stop(self):
        self._cmd(0.0, 0.0)

    def _handle_odom(self):
        pose = self.odom_node.get_pose()
        if pose:
            x, y, theta = pose
            # Updated from original
            self.get_logger().info(f"x={x:.3f}, y={y:.3f}, theta={theta:.3f} rad")


    def _handle_ir(self):
        readings = self.ir_node.get_readings()
        if readings:
            # Updated from original
            readings_str = "  ".join(f"{v:5.1f}" for v in readings)
            self.get_logger().info(f"Left->Right IR readings: {readings_str}")
        else:
            # Updated from original
            self.get_logger().info("No IR data yet.")

def main(args=None):
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

    # Build polled nodes; Controller polls odometry and IR nodes
    odom_node = OdomNode()
    ir_node = IrNode()

    # Build Controller
    # Updated from original
    controller = Controller(odom_node=odom_node, ir_node=ir_node, pulse_htz=1.0)

    # Build callback nodes; they call Controller callbacks on their respective events
    button_node = ButtonNode(controller.power_button_handler, demo_mode=False)
    hazard_node = HazardNode(controller.hazard_handler)

    # Node executor: idiomatic when spinning up multiple nodes in a single process
    executor = MultiThreadedExecutor()
    # Updated from original
    nodes = [controller, button_node, hazard_node, odom_node, ir_node]
    for node in nodes:
        executor.add_node(node)

    # Launch nodes and cleanup on keyboard interrupt
    try:
        executor.spin()
    except KeyboardInterrupt:
        logger.info("Recieved keyboard interrupt. Halting.")
    finally:
        for node in nodes:
            node.destroy_node()

        if rclpy.ok():  # likely closed already, but idiomatic to check and try
            rclpy.shutdown()


if __name__ == '__main__':
    main()
