# hl 2025 csce274 proj2a
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

''' states for state machine based on rubric '''
INITIALIZED = 0
SPIRAL_SEARCH = 1 # activates immediately on spiral search, but does not always stay active
ALIGN_WALL = 2 # goal here is to maximize right IR beam.
FOLLOW_WALL = 3 # recommended I implement PD-controller to maintain IR intensity.
MAKE_UTURN = 4
RETURN = 5


class Controller(Node):
    def __init__(self, odom_node, ir_node, pulse_htz=10.0):
       
        # Updated from original
        super().__init__('Controller')
        self.odom_node = odom_node
        self.ir_node = ir_node

        # now we are adding "stateful" machine, thus the original state is
        self.state = INITIALIZED     

        # cmd publisher to drive robot when button presses are detected
        self.cmd_vel_pub = self.create_publisher(
                Twist,      # Publishes Twist messages
                'cmd_vel',  # Topic to which its subscribers subscribe
                10)         # message buffer (for network congestion)
        
        # Updated from original
        self.dt = 1.0 / float(pulse_htz)
        self.create_timer(self.dt, self._pulse) # fixed timing issue... this is what lets robot update.
        self.get_logger().info(f"Pulse started, calling pulse() every {pulse_htz}s.")

        self.spiral = False # it is false on init. It should turn true on power button press.
        self.spi_dir = 1.0 # start on 1 because spiral is TRUE anyway on init, also it gets multiplied later in w update equation
        self.spi_count = 0
        self.spiral_scale = 1.0
        self.max_count = int(88 * self.spiral_scale)  # optional stop threshold

	    # debouncing ("filtering user input before triggering the action")
        self.debounce_s = 0.25
        self.last_button_time = -1e8
        
        # statefulness
        self.bumped = False
        self.wall_detect_ir = 40
        self.aslign_epsilon = 10 #closeness in IR units
        
        self.wall_target_ir = 80 # desired right ir intensity
        self.wall_kp = 0.002 # proportional gain
        
        # u-turn/return; we'll need to know this when the state changes
        self.state_entry_pose = None # x,y, theta
        
        
        
    # this is called on interface_buttons within ButtonNode
    def power_button_handler(self):
        # Updated from original
        self.get_logger().info("Power Button Detected")
        now = self.get_clock().now().nanoseconds *1.e-9
        if now - self.last_button_time < self.debounce_s:
            return
        self.last_button_time = now # update

        if self.state == INITIALIZED: # first time being turned on
            self.state = SPIRAL_SEARCH
            self.spiral = True # we want to spiral on init.
            self.spi_dir = 1.0
            self.spi_count=0
            
            self.state_entry_pose = self.odom_node.get_pose()
        else: # if not JUST Initialized, we have different handling... halt and reset to initilaized
            self.state = INITIALIZED
            self.spiral = False
            self.spi_count = 0 # we restarted
            self._stop() # kept stopped for now
            
            self.bumped = False
            self._stop()
            
        
    def hazard_handler(self):
        # Updated from original
        self.get_logger().info("Hazard Detected")
        self.spiral = False # stop spiral active
        self._stop()
        
        # ----------------------------------------------------------------------
        if self.state == SPIRAL_SEARCH:
            self.state = ALIGN_WALL
            self.spiral = False
            self.spi_count = 0
            self.state_entry_pose = self.odom_node.get_pose()

        elif self.state == FOLLOW_WALL or self.state == MAKE_UTURN:
            # For now: just back off slightly & let the state handler recover
            self.get_logger().info("Bump during FOLLOW_WALL/MAKE_UTURN; will attempt recovery.")
            # you can add backup motion here later if needed

        elif self.state == RETURN:
            self.get_logger().info("Bump during RETURN: resetting to INITIALIZED.")
            self.state = INITIALIZED
            self.spiral = False
       # ------------------------------------------------------------------------

        
    # Updated from original
    def _pulse(self):
        self._handle_odom()
        readings = self.ir_node.get_readings() # grab readings
        self._handle_ir()
        
        
        ''' stateful ''' 
        if self.state == INITIALIZED:
            self._stop()
        elif self.state == SPIRAL_SEARCH:
            self._state_spiral_search(readings) # takes readings from ir 
        elif self.state == ALIGN_WALL:
            self._state_align_wall(readings)
        elif self.state == FOLLOW_WALL: 
            self._state_follow_wall(readings)
        elif self.state == MAKE_UTURN:
            self._state_make_uturn()
        elif self.state == RETURN:
            self._state_return()
        else:
            pass # other states written here
        ''' end of stateful '''
        
        ''' # old code 
        if self.spiral:
        	self._do_spiral() # self.spiral() vs self._do_spiral()
        else:
            self._stop() # stay stopped, we keep this published so it doesn't start up again
        '''
    
    def _state_spiral_search(self, readings):
        # Continue spiral
        self._do_spiral()

        # Simple “found wall” test: if any front-ish sensor is bright enough
        if readings:
            # approximate front-ish indices (center-left-front, center, center-right-front)
            front_vals = readings[2:5]
            if any(v >= self.wall_detect_ir for v in front_vals):
                self.get_logger().info("Wall detected during SPIRAL_SEARCH -> ALIGN_WALL")
                self.state = ALIGN_WALL
                self.spiral = False
                self.spi_count = 0
                self.state_entry_pose = self.odom_node.get_pose()

        
    # spiral motion
    def _do_spiral(self):
        
        # todo ...
        self.spi_count += 1 # counter for tracking
        v = 0.03 + (self.spi_count * 0.033)
        w = (0.66 + (0.185 * self.spiral_scale)) * self.spi_dir

        # sanity caps
        v = max(0.03, min(v, 0.30))
        w = max(-1.2, min(w, 1.2))

        self._cmd(v, w) # send to cmd

    def _cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def _stop(self):
        self._cmd(0.0, 0.0) # obviously just set to 0

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

    # -Build Controller-
    # Updated from original
    controller = Controller(odom_node=odom_node, ir_node=ir_node, pulse_htz=10.0)
    

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
