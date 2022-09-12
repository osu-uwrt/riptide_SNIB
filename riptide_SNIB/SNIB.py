# Welcome to SNIB
import rclpy
from rclpy.node import Node

class SNIB(Node):

    def __init__(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
