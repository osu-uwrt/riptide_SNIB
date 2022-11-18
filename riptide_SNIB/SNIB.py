import os
import xacro
from threading import Timer
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped
from riptide_msgs2.msg import Depth, FirmwareState, ControllerCommand
from sensor_msgs.msg import Imu
from std_msgs .msg import Float32MultiArray, Header
from nav_msgs.msg import Odometry

from riptide_msgs2.srv import GetRobotXacro

from riptide_SNIB import simulinkControl, simulinkDataVisuals

#keep this import as last -- interesting errors pop up otherwise
import matlab.engine



VISUALS = False # Wether to show ekf and simulation positions
BUFFER_TIME = 3 # Number of seconds between bringup and actually starting simulation

class SNIB(Node):

    local_simulink = True # whether or not to launch the simulink simulation locally
    launch_unity = True #wether or not to launch unity locally
    matlab_engine = None # the matlab engine running simulink
    data_visuals_engine = None # the data visualization engine

    started_ekf = False # has the dvl been sent its inital twist

    def __init__(self):
        super().__init__('riptide_SNIB')

        # Publishers
        '''Sensor data (to filter)'''
        self.depth_pub = self.create_publisher(Depth, "depth/raw", qos_profile_sensor_data)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "dvl/twist", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "vectornav/imu", qos_profile_sensor_data)

        #thruster publishers
        self.thruster_stamp_pub = self.create_publisher(Header, "simulation/thruster_stamp", qos_profile_system_default)
        self.thruster_forces_pub = self.create_publisher(Float32MultiArray, "simulation/thruster_forces", qos_profile_system_default)

        #One time publisher for starting position at 0s
        self.initial_position_pub = self.create_publisher(PoseWithCovarianceStamped, "set_pose", qos_profile_system_default)

        #Fake dvl twist publisher
        self.initial_twist_pub = self.create_publisher(Odometry, "simulation/twist", qos_profile_system_default)
        
        #pretend that the kill switch is in
        self.firmware_state_pub = self.create_publisher(FirmwareState, "state/firmware", qos_profile_system_default)

        #control the controllers
        self.controller_linear_state_pub = self.create_publisher(ControllerCommand, "/tempest/controller/linear", qos_profile_system_default)
        self.controller_angular_state_pub = self.create_publisher(ControllerCommand, "/tempest/controller/angular", qos_profile_system_default)

        # Subscribers
        '''Simulator pose (from Simulink)'''
        self.sim_pose_sub = self.create_subscription(PoseStamped, "simulator/pose", self.sim_pose_callback, qos_profile_sensor_data)

        '''Sensor data (from Simulink)'''
        self.sim_dvl_sub = self.create_subscription(TwistStamped, "simulator/twist", self.dvl_callback, qos_profile_sensor_data)
        self.sim_imu_sub = self.create_subscription(Imu, "simulator/imu", self.imu_callback, qos_profile_sensor_data)

        self.thruster_forces_sub = self.create_subscription(Float32MultiArray, "thruster_forces", self.thruster_callback, qos_profile_system_default)

        #create a service to load the robot xacro data -- gazebo uses by way of bridge
        self.sim_xacro_loader_service = self.create_service(GetRobotXacro, "load_xacro", self.load_robot_xacro_service_cb)

        # show expected pose
        if(VISUALS):
            self.odometry_sub = self.create_subscription(Odometry, "odometry/filtered", self.odometry_cb, qos_profile_system_default)
            self.data_visuals_engine = simulinkDataVisuals.visualizationManager()

        self.publishEnabledFirmwareState()
        self.publish_initial_controller_state(3)

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

        if(self.local_simulink):
            #begin actual sim starting process

            physicalStartTimer = Timer(BUFFER_TIME, self.beginSimulationPeriod)

            #wait for the engine to load
            simulinkControl.getSimulationStatus(self.matlab_engine)
            
            physicalStartTimer.start()

            self.get_logger().info(f"Beginning physical simulation in {BUFFER_TIME} seconds!")


    def sim_pose_callback(self, msg):
        depth_msg = Depth()

        time_stamp = msg.header.stamp

        depth_variance = 0.1

        depth_msg.header.stamp = time_stamp
        depth_msg.header.frame_id = "tempest/pressure_link"
        
        depth_msg.depth = msg.pose.position.z
        depth_msg.variance = depth_variance

        simulink_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 
        ros_time = time.time()

        #ensure there is only a small difference between synconized times
        if((simulink_time - ros_time)**2 < 1):
            self.depth_pub.publish(depth_msg)
        else:
            self.publish_stamp_message()


        if(VISUALS):
            self.get_logger().info(f"Simulink time {simulink_time}.... ROS time {ros_time}")

            if(simulink_time < 1000000):
                return

            self.data_visuals_engine.append_sim_pose_data(simulink_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        
    def imu_callback(self, msg):
        imu_msg = Imu()

        time_stamp = msg.header.stamp

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

        simulink_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 
        ros_time = time.time()

        #ensure there is only a small difference between synconized times
        if((simulink_time - ros_time)**2 < 1):
            self.imu_pub.publish(imu_msg)

    def dvl_callback(self, msg):
        dvl_msg = TwistWithCovarianceStamped()

        time_stamp = msg.header.stamp

        #TODO: This should be loaded from a parameter when this node starts running, not every callback.
        cov_matrix = [1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0,
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0, 
                      1.0, 1.0, 1.0, 1.0, 1.0 ,1.0,]

        dvl_msg.header.stamp = time_stamp
        dvl_msg.header.frame_id = "tempest/base_link"
        dvl_msg.twist.covariance = cov_matrix
        dvl_msg.twist.twist = msg.twist

        if not self.started_ekf:
            #start ekf -- it doesn't start until dvl has twist

            #self.publishInitalTwist()
            self.started_ekf = True

        simulink_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 
        ros_time = time.time()

        #ensure there is only a small difference between synconized times
        if((simulink_time - ros_time)**2 < 1):
            self.dvl_pub.publish(dvl_msg)

    def thruster_callback(self, msg):
        #ensure that matlab knows exactly when the thruster forces are published
        #echo thruster forces to simulinl

        stamp_msg = Header()
        time_stamp = self.get_clock().now().to_msg()

        stamp_msg.stamp = time_stamp
        stamp_msg.frame_id = "world"

        self.thruster_stamp_pub.publish(stamp_msg)
        self.thruster_forces_pub.publish(msg)

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

    def odometry_cb(self, msg):
        time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000

        if(time < 1000000):
            return

        self.data_visuals_engine.append_ekf_pose_data(time, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

    def beginSimulationPeriod(self):
        self.get_logger().info("Beginning Physical Simulation Startup!")

        #Tell Matlab to start
        success = simulinkControl.startSimulation(self.matlab_engine)
        if not success:
            self.get_logger().error("Failed to Launch Matlab Simulation!")
            return
    
        self.get_logger().info("Physical Simulation has begun!")

    def publish_initial_controller_state(self, state):
        #state
        # 0 - disabled
        # 1 - feedforward
        # 2 - velocity
        # 3 - position

        linear_msg = ControllerCommand()
        angular_msg = ControllerCommand()

        linear_msg.mode = state
        angular_msg.mode = state

        if state == 3:
            #set the position to be underwater... zero is a hard position to maintain
            linear_msg.setpoint_vect.z = -1.0  
        else:
            linear_msg.setpoint_vect.z = 0.0

        angular_msg.setpoint_vect.z = 0.0
        linear_msg.setpoint_vect.x = 0.0
        angular_msg.setpoint_vect.x = 0.0
        linear_msg.setpoint_vect.y = 0.0
        angular_msg.setpoint_vect.y = 0.0

        angular_msg.setpoint_quat.x = 0.0
        angular_msg.setpoint_quat.y = 0.0
        angular_msg.setpoint_quat.z = 0.0
        angular_msg.setpoint_quat.w = 1.0

        linear_msg.setpoint_quat.x = 0.0
        linear_msg.setpoint_quat.y = 0.0
        linear_msg.setpoint_quat.z = 0.0
        linear_msg.setpoint_quat.w = 1.0

        self.controller_angular_state_pub.publish(angular_msg)
        self.controller_linear_state_pub.publish(linear_msg)

    def publish_stamp_message(self):
        stamp_msg = Header()
        time_stamp = self.get_clock().now().to_msg()

        stamp_msg.stamp = time_stamp
        stamp_msg.frame_id = "world"

        self.thruster_stamp_pub.publish(stamp_msg)

    def start_gz_sim_world(self):
        #the name of the sdf file for gazebo
        gz_world_name = "world2.sdf"

    def load_robot_xacro_service_cb(self, request, response):
        self.get_logger().info(f"Request {request.robot} xacro file!")

        if(request != "tempest"):
            self.get_logger().error(f"Cannot load xacro for {request.robot}!")

        #the call back to find process the robot xacro
        modelPath = os.path.join(
            get_package_share_directory('riptide_descriptions2'),
            'robots',
            "tempest" + '.xacro'
        )
        
        #default response
        response.data = "no data"

        try:
            #load the xacro data
            response.data = xacro.process_file(modelPath).toxml()
        except Exception as ex:
            self.get_logger().error("---------------------------------------------")
            self.get_logger().error("COULD NOT OPEN ROBOT DESCRIPTION XACRO FILE")
            self.get_logger().error(ex)
            self.get_logger().error("---------------------------------------------")

        # return the xacro data
        return response



    
              

def main(args=None):
    rclpy.init(args=args)

    node = SNIB()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
