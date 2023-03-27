import os
import xacro
from threading import Timer
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix

from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped
from riptide_msgs2.msg import Depth, ControllerCommand
from sensor_msgs.msg import Imu
from std_msgs .msg import Float32MultiArray, Header, Bool
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

    #TODO: export correct rotation from gazebo
    world_com_rot_matrix = np.eye(4) # the most recent transformation from world frame to robot frame

    com_imu_trans_matrix = [-1] # saves the frame after the first call -> this should never change 
    com_dvl_trans_matrix = [-1] # saves the frame after the first call -> this should never change 
    com_pressure_rot_matrix = [-1]
    com_pressure_translation = [-1]

    def __init__(self):
        super().__init__('riptide_SNIB')

        # Publishers
        '''Sensor data (to filter)'''
        self.depth_pub = self.create_publisher(Depth, "/talos/depth/raw", qos_profile_sensor_data)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "/talos/dvl_twist", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/talos/vectornav/imu", qos_profile_sensor_data)

        #thruster publishers
        self.thruster_stamp_pub = self.create_publisher(Header, "/talos/simulation/thruster_stamp", qos_profile_system_default)
        self.thruster_forces_pub = self.create_publisher(Float32MultiArray, "/talos/simulation/thruster_forces", qos_profile_system_default)

        #One time publisher for starting position at 0s
        self.initial_position_pub = self.create_publisher(PoseWithCovarianceStamped, "/talos/set_pose", qos_profile_system_default)

        #Fake dvl twist publisher
        self.initial_twist_pub = self.create_publisher(Odometry, "/talos/simulation/twist", qos_profile_system_default)
        
        #pretend that the kill switch is in
        self.kill_state_pub = self.create_publisher(Bool, "/talos/state/kill", qos_profile_system_default)

        #control the controllers
        self.controller_linear_state_pub = self.create_publisher(ControllerCommand, "/talos/controller/linear", qos_profile_system_default)
        self.controller_angular_state_pub = self.create_publisher(ControllerCommand, "/talos/controller/angular", qos_profile_system_default)

        # Subscribers
        '''Simulator pose (from Simulink)'''
        self.sim_pose_sub = self.create_subscription(PoseStamped, "/talos/simulator/pose", self.sim_pose_callback, qos_profile_sensor_data)

        '''Sensor data (from Simulink)'''
        self.sim_dvl_sub = self.create_subscription(TwistStamped, "/talos/simulator/twist", self.dvl_callback, qos_profile_sensor_data)
        self.sim_imu_sub = self.create_subscription(Imu, "/talos/simulator/imu", self.imu_callback, qos_profile_sensor_data)

        self.thruster_forces_sub = self.create_subscription(Float32MultiArray, "/talos/thruster_forces", self.thruster_callback, qos_profile_system_default)


        #create a service to load the robot xacro data -- gazebo uses by way of bridge
        self.sim_xacro_loader_service = self.create_service(GetRobotXacro, "/talos/load_xacro", self.load_robot_xacro_service_cb)


        # show expected pose
        if(VISUALS):
            self.odometry_sub = self.create_subscription(Odometry, "/talos/odometry/filtered", self.odometry_cb, qos_profile_system_default)
            self.data_visuals_engine = simulinkDataVisuals.visualizationManager()


        #initialize things
        self.publishEnabledFirmwareState()
        self.publish_initial_controller_state(3)


        #initialize tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # start up matlab
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
                session_names = []
                break

        if (len(session_names) != 0):
            for name in session_names:
                #try all of the names
                if not self.matlab_engine:
                    #if the engine hasn't been found yet.. only need one
                    if "SNIBulink" in name:
                        #only use the engine if designated for simulation use

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

            #stop to ensure a clean restart
            simulinkControl.stopSimulation(self.matlab_engine)
            
            physicalStartTimer.start()

            self.get_logger().info(f"Beginning physical simulation in {BUFFER_TIME} seconds!")


    def sim_pose_callback(self, msg):
        depth_msg = Depth()

        time_stamp = msg.header.stamp

        #must actually be in imu frame - transform from com frame
        com_frame_name = "talos/base_inertia"
        depth_frame_name = "talos/pressure_link"

        try:
            # if the imu com transform hasnt been found yet look it up
            if len(self.com_pressure_translation) == 1:
                com_pressure_transform = self.tf_buffer.lookup_transform(depth_frame_name, com_frame_name, rclpy.time.Time())

                self.com_pressure_translation = [
                    com_pressure_transform.transform.translation.x,
                    com_pressure_transform.transform.translation.y,
                    com_pressure_transform.transform.translation.z
                    ]

        except TransformException as exception:
            self.get_logger().info(f"Could not get transform from {com_frame_name} to {depth_frame_name}: {exception}")
            return


        rot_matrix = quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[:3, :3]
        absolute_translation = np.dot(np.array(rot_matrix), np.array(self.com_pressure_translation))
        
        #self.get_logger().info(f"relative{self.com_pressure_translation}, absolute{absolute_translation}")


        depth_variance = 0.0000001

        depth_msg.header.stamp = time_stamp
        depth_msg.header.frame_id = "talos/pressure_link"

        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] + absolute_translation
        depth_msg.depth = position[2]
        depth_msg.variance = depth_variance

        self.get_logger().info("Depth Z: " + str(position[2]))

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

    def fetch_imu_transformation_matrix(self) -> bool:
        # fetch the dvl transformation matrix from tf

        #must actually be in imu frame - transform from com frame
        com_frame_name = "talos/base_inertia"
        imu_frame_name = "talos/imu_link"
        
        try:
            # if the imu com transform hasnt been found yet look it up
            if len(self.com_imu_trans_matrix) == 1:
                com_imu_transform = self.tf_buffer.lookup_transform(imu_frame_name, com_frame_name, rclpy.time.Time())

                self.com_imu_quaternion =[
                    com_imu_transform.transform.rotation.x,
                    com_imu_transform.transform.rotation.y,
                    com_imu_transform.transform.rotation.z,
                    com_imu_transform.transform.rotation.w,
                    ]
                
                self.com_imu_trans_matrix = quaternion_matrix(self.com_imu_quaternion)
                
                #fill out translation
                self.com_imu_trans_matrix[0][3] = float(com_imu_transform.transform.translation.x)
                self.com_imu_trans_matrix[1][3] = float(com_imu_transform.transform.translation.y)
                self.com_imu_trans_matrix[2][3] = float(com_imu_transform.transform.translation.z)

            return True
        except TransformException as exception:
            self.get_logger().info(f"Could not get transform from {com_frame_name} to {imu_frame_name}: {exception}")
            return False

    def fetch_dvl_transformation_matrix(self) -> bool:
        # fetch the dvl transformation matrix from tf

        #must actually be in imu frame - transform from com frame
        com_frame_name = "talos/base_inertia"
        dvl_frame_name = "talos/dvl_link"
        
        try:
            # if the imu com transform hasnt been found yet look it up
            if len(self.com_dvl_trans_matrix) == 1:
                com_dvl_transform = self.tf_buffer.lookup_transform(dvl_frame_name, com_frame_name, rclpy.time.Time())

                self.com_dvl_quaternion =[
                    com_dvl_transform.transform.rotation.x,
                    com_dvl_transform.transform.rotation.y,
                    com_dvl_transform.transform.rotation.z,
                    com_dvl_transform.transform.rotation.w,
                    ]

                self.com_dvl_trans_matrix = quaternion_matrix(self.com_dvl_quaternion)
                
                self.get_logger().info(str(float(com_dvl_transform.transform.translation.x)))

                #fill out translation
                self.com_dvl_trans_matrix[0][3] = float(com_dvl_transform.transform.translation.x)
                self.com_dvl_trans_matrix[1][3] = float(com_dvl_transform.transform.translation.y)
                self.com_dvl_trans_matrix[2][3] = float(com_dvl_transform.transform.translation.z)

            return True
        except TransformException as exception:
            self.get_logger().info(f"Could not get transform from {com_frame_name} to {dvl_frame_name}: {exception}")
            return False

    def imu_callback(self, msg):
        imu_msg = Imu()

        time_stamp = msg.header.stamp

        if not self.fetch_imu_transformation_matrix():
            return


        #TODO: These should be loaded from a parameter when this node starts running, not every callback.
        orientation_cov_matrix = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        angular_vel_cov_matrix = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
        linear_acc_cov_matrix  = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = "talos/imu_link"


        #calculate the absolute orientation of the imu in world frame
        self.world_com_rot_matrix = quaternion_matrix([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        imu_transformation_matrix = np.matmul(self.world_com_rot_matrix, self.com_imu_trans_matrix)
        orientation = quaternion_from_matrix(imu_transformation_matrix)
        euler = euler_from_matrix(self.world_com_rot_matrix)

        self.get_logger().info("Imu roll: " + str(euler[0]) + " Imu pitch: " + str(euler[1]) + " Imu yaw: " + str(euler[2]))

        imu_msg.orientation.x = orientation[0]
        imu_msg.orientation.y = orientation[1]
        imu_msg.orientation.z = orientation[2]
        imu_msg.orientation.w = orientation[3]
        imu_msg.orientation_covariance = orientation_cov_matrix

        imu_orientation_matrix = imu_transformation_matrix[0:3,0:3]
        angular_velocity = np.dot(imu_orientation_matrix, np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))

        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        imu_msg.angular_velocity_covariance = angular_vel_cov_matrix

        linear_acceleration = np.matmul(imu_orientation_matrix, np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,msg.linear_acceleration.z]))
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        imu_msg.linear_acceleration_covariance = linear_acc_cov_matrix

        simulink_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 
        ros_time = time.time()

        #ensure there is only a small difference between synconized times
        if((simulink_time - ros_time)**2 < 1):
            self.imu_pub.publish(imu_msg)
        else:
            self.get_logger().error(f"Not publishing IMU because SNIB and simulink are not in sync!\nRos time is: {ros_time} and Simulink time is: {simulink_time}.")
        
    def dvl_callback(self, msg):
        dvl_msg = TwistWithCovarianceStamped()
        time_stamp = msg.header.stamp

        if not self.fetch_dvl_transformation_matrix():
            return

        #TODO: This should be loaded from a parameter when this node starts running, not every callback.
        #Don't Trust DVL Twist angular
        cov_matrix = [0.00001, 0.0, 0.0, 0.0, 0.0 ,0.0, 
                      0.0, 0.00001, 0.0, 0.0, 0.0 ,0.0,
                      0.0, 0.0, 0.00001, 0.0, 0.0 ,0.0, 
                      0.0, 0.0, 0.0, 1000.0, 0.0 ,0.0, 
                      0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 
                      0.0, 0.0, 0.0, 0.0, 0.0, 1000.0,]

        #dvl msg header and cov
        dvl_msg.header.stamp = time_stamp
        dvl_msg.header.frame_id = "talos/base_link"
        dvl_msg.twist.covariance = cov_matrix
        
        #calculate linear velocities in dvl frame
        world_dvl_trans_matrix = np.matmul(self.world_com_rot_matrix, self.com_dvl_trans_matrix)
        dvl_velocities = np.dot(world_dvl_trans_matrix, np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, 1]))
        dvl_msg.twist.twist.linear.x = dvl_velocities[0]
        dvl_msg.twist.twist.linear.y = dvl_velocities[1]
        dvl_msg.twist.twist.linear.z = dvl_velocities[2]  

        #calculate angular dvl velocities
        dvl_angular_velocities = np.dot(world_dvl_trans_matrix, np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z, 1]))  
        dvl_msg.twist.twist.angular.x = dvl_angular_velocities[0]
        dvl_msg.twist.twist.angular.y = dvl_angular_velocities[1]
        dvl_msg.twist.twist.angular.z = dvl_angular_velocities[2]

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
        kill_state_msg = Bool()

        kill_state_msg.data = True

        self.kill_state_pub.publish(kill_state_msg)

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
            linear_msg.setpoint_vect.z = -6.0  
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

        if(request != "talos"):
            self.get_logger().error(f"Cannot load xacro for {request.robot}!")

        #the call back to find process the robot xacro
        modelPath = os.path.join(
            get_package_share_directory('riptide_descriptions2'),
            'robots',
            "talos" + '.xacro'
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
