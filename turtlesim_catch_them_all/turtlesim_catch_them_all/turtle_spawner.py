#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
import random
from user_interfaces.msg import NewTurtle
from turtlesim.msg import Pose

NODE_NAME = "turtle_spawner"
SPAWN_TIME = 1.5  # spawns/creates a turtle in every 1 second

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.spawn_client_ = self.create_client(Spawn, '/spawn')
        self.turtle_count_ = 1
        self.turtle_name = ""
        self.kill_turtle_name = ""
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.new_turtle_publisher_ = self.create_publisher(NewTurtle, '/new_turtle_avail', 10)
        self.create_timer(SPAWN_TIME, self.call_spawn_turtle)
        self.kill_turtle_service_ = self.create_client(Kill, '/kill')
        self.pose_turtle1_subs_ = self.create_subscription(Pose, '/turtle1/pose', self.kill_turtle, 10)

    def kill_turtle(self, msg):
        if msg is not None:
            dist = math.sqrt((msg.x - self.turtle_x)**2 + (msg.y - self.turtle_y)**2)
            if dist < 1:  # if turtle is close enough to the target
                request = Kill.Request()
                if self.turtle_name != "":
                    request.name = self.turtle_name
                    self.kill_turtle_name = self.turtle_name
                    future = self.kill_turtle_service_.call_async(request)
                    future.add_done_callback(self.callback_kill)
                else:
                    pass # No turtle to kill

    def callback_kill(self, future):
        response = future.result()
        self.get_logger().info(f"Turtle killed, {self.kill_turtle_name} success")

    def call_spawn_turtle(self):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim Spawn service..")

        request = Spawn.Request()
        self.turtle_x = random.uniform(0, 11)
        request.x = self.turtle_x
        self.turtle_y = random.uniform(0, 11)
        request.y = self.turtle_y
        request.theta = random.uniform(0, 2 * math.pi)
        self.turtle_name = "turtle_" + str(self.turtle_count_)
        request.name = self.turtle_name
        self.turtle_count_ += 1

        self.publish_new_turtle()

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(self.callback_spawn)

    def callback_spawn(self, future):
        response = future.result()
        if response is not None and response.name != "":
            self.get_logger().info(f"Turtle spawned: {response.name}, success")

    def publish_new_turtle(self):
        msg = NewTurtle()
        msg.name = self.turtle_name
        msg.x = self.turtle_x
        msg.y = self.turtle_y
        self.new_turtle_publisher_.publish(msg)
        
def main():
    rclpy.init()
    node = TurtleSpawnerNode()
    node.call_spawn_turtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()