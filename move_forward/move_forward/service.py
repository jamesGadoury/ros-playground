from example_interfaces.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.clock import Clock
from geometry_msgs.msg import Twist, Vector3

class MoveForwardService(Node):
    def __init__(self):
        super().__init__("move_forward_service")
        # TODO use our own interface
        self.srv = self.create_service(Trigger, "trigger", self.callback)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.clock = Clock()

    def callback(self, request, response):
        self.get_logger().info(f"Incoming request {request}")
        msg = Twist(linear=Vector3(x=0.1))
        self.get_logger().info(f"Publishing move forward msg {msg}")
        self.publisher.publish(msg)

        time_to_wait = Duration(seconds=10)
        self.get_logger().info(f"Waiting...{time_to_wait}")

        start = self.clock.now()
        while self.clock.now() - start < time_to_wait: pass

        msg = Twist()
        self.get_logger().info(f"Publishing stop msg {msg}")
        self.publisher.publish(msg)

        self.get_logger().info(f"Done...")
        response.success = True
        return response
    

def main():
    rclpy.init()

    service = MoveForwardService()
    rclpy.spin(service)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
