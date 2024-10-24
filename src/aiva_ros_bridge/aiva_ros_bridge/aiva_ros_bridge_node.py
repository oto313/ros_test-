import threading
import rclpy
from rclpy.node import Node


class AivaRosBridgeNode(Node):
    def __init__(self):
        super().__init__('aiva_ros_bridge')
        self.thread = threading.Thread(target=self.thread, args=(self,))

    def shutdown(self):
        rclpy.shutdown()
    def thread(node):
        rclpy.spin(node)