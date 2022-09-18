import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# Message types
from geometry_msgs.msg import PoseWithCovarianceStamped

class SNIB(Node):

    def __init__(self):
        super().__init__('riptide_SNIB')
        self.depth_pub = self.create_publisher(PoseWithCovarianceStamped, "depth/pose", qos_profile_sensor_data)

        #TODO: Create dvl publisher

def main(args=None):
    rclpy.init(args=args)

    node = SNIB()

    # Spoof depth 
    depth_msg = PoseWithCovarianceStamped()
    depth_msg.header.stamp = node.get_clock().now()
    depth_msg.header.frame_id = "odom"
    depth_msg.pose.pose.position.z = -10
    depth_msg.pose.covariance[14] = 1e-3

    node.depth_pub.publish(depth_msg)

    # Spoof DVL

    #TODO: Build message

    #TODO: Publish message

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
