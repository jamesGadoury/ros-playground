from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node

class MoveForwardClientAsync(Node):
    def __init__(self):
        super().__init__("move_forward_client_async")
        self.cli = self.create_client(Trigger, "trigger")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main():
    rclpy.init()

    client = MoveForwardClientAsync()
    response = client.send_request()
    client.get_logger().info(f"response: {response}")
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

