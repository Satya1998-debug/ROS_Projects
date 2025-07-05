import rclpy
import time
from rclpy.node import Node
from ros_interfaces.srv import LedControl

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery')
        self.battery_state = 100  # Battery level in percentage
        self.battery_client_ = self.create_client(LedControl, 'set_led')
        self.get_logger().info('Battery Node has been started.')
        self.get_logger().info('Client "set_led" is ready to send requests.')

    def toggle_battery(self):
        while True:
            time.sleep(4)
            self.battery_state = 0
            self.send_battery_status()
            time.sleep(6)
            self.battery_state = 100
            self.send_battery_status()

    def send_battery_status(self):
        if not self.battery_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
            return

        self.led_control = LedControl.Request()
        if self.battery_state == 0:
            self.led_control.led_number = 3
            self.led_control.state = "ON"
            self.get_logger().info("Calling LED server: Battery is empty, turning on LED 3.")
            
        else:
            self.led_control.led_number = 0
            self.led_control.state = "OFF"
            self.get_logger().info("Calling LED server: Battery is full, turning OFF LED 3.")

        future = self.battery_client_.call_async(self.led_control)
        future.add_done_callback(self.callback_battery_status)
        
    def callback_battery_status(self, future):
        try:
            response = future.result()
            self.get_logger().warn(f"{str(response.message)}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryNode()
    battery_node.toggle_battery()
    rclpy.spin(battery_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
