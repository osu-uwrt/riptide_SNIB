from logging import Logger
from click import launch
from numpy import empty
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import Pose, Twist, TwistWithCovarianceStamped
from riptide_msgs2.msg import Depth, FirmwareState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matlab.engine
from riptide_SNIB import simulinkControl
import numpy as np
import time

class SNIB(Node):

    local_simulink = True # whether or not to launch the simulink simulation locally
    launch_unity = True #wether or not to launch unity locally
    matlab_engine = None # the matlab engine running simulink

    def __init__(self):
        super().__init__('riptide_SNIB')

        # Publishers
        '''Sensor data (to filter)'''
        self.depth_pub = self.create_publisher(Depth, "depth/raw", qos_profile_sensor_data)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "dvl_twist", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "imu/imu/data", qos_profile_sensor_data)

        #One time publisher for starting position at 0s
        self.initial_position_pub = self.create_publisher(Odometry, "odometry/filtered", qos_profile_system_default)
        
        #pretend that the kill switch is in
        self.firmware_state_pub = self.create_publisher(FirmwareState, "state/firmware", qos_profile_system_default)

        # Subscribers
        '''Simulator pose (from Simulink)'''
        self.sim_pose_sub = self.create_subscription(Pose, "simulator/pose", self.sim_pose_callback, qos_profile_sensor_data)

        '''Sensor data (from Simulink)'''
        self.sim_dvl_sub = self.create_subscription(Twist, "simulator/twist", self.dvl_callback, qos_profile_sensor_data)
        self.sim_imu_sub = self.create_subscription(Imu, "snib/imu", self.imu_callback, qos_profile_sensor_data)

        self.publishStartingPosition()
        self.publishEnabledFirmwareState()

        session_names = None
        timeout = time.time() + 60
        while(session_names == None):
            time.sleep(1)
            #find the matlab engine to run simulink
            session_names = matlab.engine.find_matlab()
            self.get_logger().info("Attempting to aquire Matlab engine")

            try:
                if len(session_names) == 0:
                    session_names = None
            except:
                session_names = None

            if(time.time() > timeout):
                break

        if (len(session_names) != 0):
            #if a local matlab session has been launch - should be done in simulation.launch.py
            try:
                self.matlab_engine = matlab.engine.connect_matlab(session_names[0])
                self.get_logger().info("Running Simulink locally on Matlab Engine: " + str(self.matlab_engine))

            except:
                self.get_logger().error("SNIB: Running Matlab session found but could not connect!")
        else:
            #if runnning a hardware in loop simulation
            self.local_simulink = False
            self.get_logger().info("The simulink portion of the simulation will not be launched locally")


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
        dvl_msg.header.frame_id = "tempest/dvl_link"
        dvl_msg.twist.covariance = cov_matrix
        dvl_msg.twist.twist = msg

        self.dvl_pub.publish(dvl_msg)

    def publishStartingPosition(self):
        #basically 20 lines of its at 0 

        initial_pos_msg = Odometry()

        time_stamp = self.get_clock().now().to_msg()
        initial_pos_msg.header.stamp = time_stamp
        initial_pos_msg.header.frame_id = "world"

        initial_pos_msg.pose.pose.position.x = 0.0
        initial_pos_msg.pose.pose.position.y = 0.0
        initial_pos_msg.pose.pose.position.z = 0.0

        initial_pos_msg.pose.pose.orientation.w = 1.0
        initial_pos_msg.pose.pose.orientation.x = 0.0
        initial_pos_msg.pose.pose.orientation.y = 0.0
        initial_pos_msg.pose.pose.orientation.z = 0.0

        initial_pos_msg.pose.covariance = np.zeros(shape=(36))

        initial_pos_msg.twist.twist.linear.x = 0.0
        initial_pos_msg.twist.twist.linear.y = 0.0
        initial_pos_msg.twist.twist.linear.z = 0.0

        initial_pos_msg.twist.twist.angular.x = 0.0
        initial_pos_msg.twist.twist.angular.y = 0.0
        initial_pos_msg.twist.twist.angular.z = 0.0

        initial_pos_msg.twist.covariance = np.zeros(shape=(36))

        self.initial_position_pub.publish(initial_pos_msg)

    def publishEnabledFirmwareState(self):
        firmware_state_msg = FirmwareState()

        time_stamp = self.get_clock().now().to_msg()
        firmware_state_msg.header.stamp = time_stamp
        firmware_state_msg.header.frame_id = "world"

        firmware_state_msg.version_major = 0
        firmware_state_msg.version_minor = 0

        firmware_state_msg.depth_sensor_initialized = False
        firmware_state_msg.actuator_connected = False

        firmware_state_msg.actuator_faults = 0
        firmware_state_msg.peltier_cooling_threshold = 0
        firmware_state_msg.copro_faults = 0
        firmware_state_msg.copro_memory_usage = 0

        # pretend the kill switch is in
        firmware_state_msg.kill_switches_enabled = 1

        firmware_state_msg.kill_switches_asserting_kill = 0
        firmware_state_msg.kill_switches_needs_update = 0
        firmware_state_msg.kill_switches_timed_out = 0

        self.firmware_state_pub.publish(firmware_state_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SNIB()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
