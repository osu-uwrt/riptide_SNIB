import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Message types
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class SNIB(Node):

    def __init__(self):
        super().__init__('riptide_SNIB')

        # Publishers
        self.depth_pub = self.create_publisher(PoseWithCovarianceStamped, "depth/pose", qos_profile_sensor_data)
        # TODO: Create dvl publisher
        # TODO: Create imu publisher

        # Subscribers
        self.sim_dvl_sub = self.create_subscription(PoseStamped, "snib/dvl", self.dvl_callback, qos_profile_sensor_data)
        self.sim_depth_sub = self.create_subscription(PoseStamped, "snib/depth", self.depth_callback, qos_profile_sensor_data)
        self.sim_imu_sub = self.create_subscription(PoseStamped, "snib/imu", self.imu_callback, qos_profile_sensor_data)

        self.rate = self.create_rate(2)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def imu_callback(self):
        pass

    def dvl_callback(self):
        pass

    def depth_callback(self):
        pass


    def timer_callback(self):
        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "odom"
        depth_msg.pose.pose.position.z = -10.0
        depth_msg.pose.covariance[14] = 1.0e-3

        self.depth_pub.publish(depth_msg)

        self.get_logger().info('Publishing...')
        self.i += 1


    def dummy_callback():
        pass

def main(args=None):
    rclpy.init(args=args)

    node = SNIB()
    

    # Spoof DVL

    #TODO: Build message

    #TODO: Publish message

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
