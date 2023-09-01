from example_interfaces.srv import Trigger

import rclpy
from rclpy.node import Node

class MoveForwardService(Node):
    def __init__(self):
        super().__init__("move_forward_service")
        # TODO use our own interface
        self.srv = self.create_service(Trigger, "trigger", self.callback)
    
    def callback(self, request, response):
        self.get_logger().info(f"Incoming request {request}")
        response.success = True
        return response
    

def main():
    rclpy.init()

    service = MoveForwardService()
    rclpy.spin(service)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
