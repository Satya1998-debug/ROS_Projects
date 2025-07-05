import rclpy
from rclpy.node import Node
from ros_interfaces.msg import LedState
from ros_interfaces.srv import LedControl

class LedPanelNode(Node):

    def __init__(self):
        super().__init__('led_panel')
        self.led_server_ = self.create_service(LedControl, 'set_led', self.led_server_callback)
        self.get_logger().info('LED Panel Node has been started.')
        self.get_logger().info('Service "set_led" is ready to accept requests.')
        self.led_number = 0
        self.led_state = "OFF"  # Default state of the LED
        self.led_state_publisher_ = self.create_publisher(LedState, 'led_panel_state', 10)
        self.timer_ = self.create_timer(1.0, self.publish_led_state)

    def led_server_callback(self, request: LedControl.Request, response: LedControl.Response):
        self.led_number = request.led_number
        self.led_state = request.state

        if self.led_number == 3 and self.led_state == "ON":
            # turn on particular LED number, when battery is empty
            response.success = True
            response.message = "LED {} has been turned {}".format(str(self.led_number), str(self.led_state))
            self.get_logger().info(str(response.message))

        else:
            response.success = False
            response.message = "LED is OFF, as with Battery if FULL or LED is not operational."
            self.get_logger().info(str(response.message))
            
        # publish the LED state
        self.get_logger().info("Publishing LED state...")
        led_state_msg = LedState()
        led_state_msg.led_number = self.led_number
        led_state_msg.state = self.led_state
        # self.led_state_publisher_.publish(led_state_msg)

        return response
    
    def publish_led_state(self):
        led_state_msg = LedState()
        led_state_msg.led_number = self.led_number
        led_state_msg.state = self.led_state
        self.led_state_publisher_.publish(led_state_msg)
        self.get_logger().info(f"Published LED state: {self.led_number} - {self.led_state}")
    
def main(args=None):
    rclpy.init(args=args)
    led_panel_node = LedPanelNode()
    rclpy.spin(led_panel_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
# This code defines a ROS 2 node that provides a service to control an LED pane