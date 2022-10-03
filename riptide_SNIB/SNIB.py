import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist, TwistWithCovarianceStamped
from riptide_msgs2.msg import Depth 
from sensor_msgs.msg import Imu

class SNIB(Node):

    def __init__(self):
        super().__init__('riptide_SNIB')

        # Publishers
        '''Sensor data (to filter)'''
        self.depth_pub = self.create_publisher(Depth, "depth/raw", qos_profile_sensor_data)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "dvl_raw", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "imu/imu/data", qos_profile_sensor_data)
        
        # Subscribers
        '''Simulator pose (from Simulink)'''
        self.sim_pose_sub = self.create_subscription(Pose, "simulator/pose", self.sim_pose_callback, qos_profile_sensor_data)

        '''Sensor data (from Simulink)'''
        self.sim_dvl_sub = self.create_subscription(Twist, "simulator/twist", self.dvl_callback, qos_profile_sensor_data)
        self.sim_imu_sub = self.create_subscription(Imu, "snib/imu", self.imu_callback, qos_profile_sensor_data)

    def sim_pose_callback(self, msg):
        depth_msg = Depth()

        time_stamp = self.get_clock().now().to_msg()

        depth_variance = 0.1

        depth_msg.header.stamp = time_stamp
        depth_msg.header.frame_id = "tempest/pressure_link"
        
        depth_msg.depth = msg.position.z
        depth_msg.variance = depth_variance

        self.depth_pub.publish(depth_msg)
        
    def imu_callback(self, msg):
        imu_msg = Imu()

        time_stamp = self.get_clock().now().to_msg()

        #TODO: These should be loaded from a parameter when this node starts running, not every callback.
        orientation_cov_matrix = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        angular_vel_cov_matrix = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        linear_acc_cov_matrix  = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = "tempest/imu_link"

        imu_msg.orientation = msg.orientation
        imu_msg.orientation = orientation_cov_matrix

        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.angular_velocity_covariance = angular_vel_cov_matrix

        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = linear_acc_cov_matrix

        self.imu_pub.publish(imu_msg)

    def dvl_callback(self, msg):
        dvl_msg = TwistWithCovarianceStamped()

        time_stamp = self.get_clock().now().to_msg()

        #TODO: This should be loaded from a parameter when this node starts running, not every callback.
        cov_matrix = [1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0,
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0,]

        dvl_msg.header.stamp = time_stamp
        dvl_msg.header.frameid = "tempest/dvl_link"
        dvl_msg.twist.covariance = cov_matrix
        dvl_msg.twist.twist = msg

        self.dvl_pub.publish(dvl_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SNIB()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
