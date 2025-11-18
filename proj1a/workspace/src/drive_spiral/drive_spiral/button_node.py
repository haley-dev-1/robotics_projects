"""
ButtonNode class for iRobot Create 3
---------------------------------------
- Subscribes to "proj1/interface_buttons"
- Publishes to "proj1/spiral_cmd"
    HALT when Button Power is pressed,
    CCW when Button 1 is pressed, or
    CW Button 2 is pressed.
"""
from rclpy.node import Node
from irobot_create_msgs.msg import InterfaceButtons
from std_msgs.msg import String


class ButtonNode(Node):
    """
    Listens for a simulated button press, then publishes CCW or CW if button 1
    or 2 is pressed.
    Publishes to "spiral_cmd" node.
    """
    def __init__(self, demo_mode=False):
        super().__init__('button_node')

        self.sub = self.create_subscription(
            InterfaceButtons,
            'proj1/interface_buttons',
            self._button_callback,
            10
        )

        # Mode toggle: False = test with beeps, True = test witout
        self.demo_mode = demo_mode

        # Create publisher for drive command
        self.pub = self.create_publisher(String, 'proj1/spiral_cmd', 10)

        self.get_logger().info(
            f"ButtonNode ready. demo_mode = {self.demo_mode}"
        )

    def _button_callback(self, msg: InterfaceButtons):
        """Internal: handle raw button messages."""
        if msg.button_power.is_pressed:
            self.pub.publish(String(data="HALT"))

        elif msg.button_1.is_pressed:
            self.pub.publish(String(data="CCW"))

        elif msg.button_2.is_pressed:
            self.pub.publish(String(data="CW"))
