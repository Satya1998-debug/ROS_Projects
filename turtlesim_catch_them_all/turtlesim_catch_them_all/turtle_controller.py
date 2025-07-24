#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from user_interfaces.msg import NewTurtle
from turtlesim.msg import Pose

NODE_NAME = "turtle_controller"

class TrurtleControllerNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.new_turtle_subscriber_ = self.create_subscription(NewTurtle, '/new_turtle_avail', self.new_turtle_callback, 10)
        self.pose_turtle1_subs_ = self.create_subscription(Pose, '/turtle1/pose', self.publish_cmd_vel, 10)
        self.target_x = 0.0
        self.target_y = 0.0

    def new_turtle_callback(self, msg):
        if msg is not None:
            self.get_logger().info(f"New turtle available: {msg.name} at ({msg.x}, {msg.y})")
            self.target_x = msg.x
            self.target_y = msg.y

    def publish_cmd_vel(self, msg):
        msg_pub = Twist()
        
        # distance x to move
        self.dist = 2*math.sqrt((msg.x - self.target_x)**2 + (msg.y - self.target_y)**2)
        self.angle_to_target = math.atan2(self.target_y - msg.y, self.target_x - msg.x) - msg.theta

        # normalize angle to be within -pi to pi
        if self.angle_to_target > math.pi:
            self.angle_to_target -= 2 * math.pi
        elif self.angle_to_target < -math.pi:
            self.angle_to_target += 2 * math.pi

        if self.dist < 0.05:  # target reached
            msg_pub.linear.x = 0.0
            msg_pub.angular.z = 0.0
        else:
            msg_pub.linear.x = self.dist
            msg_pub.angular.z = 6*self.angle_to_target
        self.cmd_vel_pub_.publish(msg_pub)

def main():
    rclpy.init()
    node = TrurtleControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

 
 
if __name__ == "__main__":
    main()