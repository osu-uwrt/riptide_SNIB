# Welcome to SNIB
import rclpy
from rclpy.node import Node

class SNIB(Node):

    def __init__(self):
        super().__init__('riptide_SNIB')


def main(args=None):
    rclpy.init(args=args)

    node = SNIB()

    rclpy.sping(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
