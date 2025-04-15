# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from px4_msgs.msg import VehicleCommand, TrajectorySetpoint

# class PX4MultiDroneController(Node):
#     def __init__(self):
#         super().__init__('px4_multi_drone_controller')
#         self.num_drones = 3
#         self.parameter_set = [False] * self.num_drones
#         self.my_pub = {}
#         self.altitudes = [20.0, 50.0, 80.0]  # Different altitudes for each drone

#         for i in range(self.num_drones):
#             self.my_pub[f'vehicle_command_{i}'] = self.create_publisher(
#                 VehicleCommand, f'/px4_{i}/fmu/in/vehicle_command', 10)
#             self.my_pub[f'gimbal_pitch_{i}'] = self.create_publisher(
#                 Float64, f'/model/x500_gimbal_{i}/command/gimbal_pitch', 10)
#             self.my_pub[f'trajectory_{i}'] = self.create_publisher(
#                 TrajectorySetpoint, f'/px4_{i}/fmu/in/trajectory_setpoint', 10)

#         self.set_parameters()
#         # self.pitch_timer = self.create_timer(0.01, self.publish_pitch)
#         self.altitude_timer = self.create_timer(0.01, self.publish_altitudes)

#     def set_parameters(self):
#         for i in range(self.num_drones):
#             if not self.parameter_set[i]:
#                 msg = VehicleCommand()
#                 msg.timestamp = self.get_clock().now().nanoseconds // 1000
#                 msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE
#                 msg.param1 = 982.0
#                 msg.param2 = 4.0
#                 msg.target_system = i + 1
#                 msg.target_component = 1
#                 print("MY CHECK!!!!!!!!!!!!!!!!!!!!!!!")
#                 self.my_pub[f'vehicle_command_{i}'].publish(msg)
#                 self.get_logger().info(f"Sent parameter set command for drone {i}")
#                 self.parameter_set[i] = True

#     def publish_pitch(self):
#         for i in range(self.num_drones):
#             msg = Float64()VEHICLE_CMD_DO_CHANGE_ALTITUDE
#             msg.data = 30.0  # Same pitch for all drones
#             self.my_pub[f'gimbal_pitch_{i}'].publish(msg)

#     def publish_altitudes(self):
#         for i in range(self.num_drones):
#             msg = TrajectorySetpoint()
#             msg.position = [0.0, 0.0, -self.altitudes[i]]  # Z is negative for altitude
#             msg.yaw = 0.0
#             self.my_pub[f'trajectory_{i}'].publish(msg)
#             self.get_logger().info(f"Published altitude for drone {i}: {self.altitudes[i]}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = PX4MultiDroneController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

'''import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleControlMode  # Adjust this import if a different message type is used

class DroneAltitudeControl(Node):

    def __init__(self):
        super().__init__('drone_altitude_control')

        # Create publishers for each drone's vehicle_command topic
        self.drone_1_publisher = self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command', 10)
        self.drone_2_publisher = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10)
        self.drone_3_publisher = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10)

        # Set different target altitudes for each drone (in meters)
        self.target_altitude_drone_1 = 10.0  # Drone 1 Altitude
        self.target_altitude_drone_2 = 8.0   # Drone 2 Altitude
        self.target_altitude_drone_3 = 12.0  # Drone 3 Altitude

        # Set up a timer to send the altitude command after setting mode
        self.timer = self.create_timer(1.0, self.control_drones)  # Send command every second

        # Track if drones are armed and in offboard mode
        self.drones_armed = False
        self.drones_in_offboard_mode = False

    def control_drones(self):
        # If drones are not armed or in offboard mode, do that first
        if not self.drones_armed:
            self.arm_drones()

        elif not self.drones_in_offboard_mode:
            self.set_offboard_mode()

        else:
            # Once armed and in offboard mode, send the altitude commands
            self.send_altitude_command()

    def arm_drones(self):
        # Send the arm command to each drone
        self.get_logger().info("Arming drones...")
        
        arm_command = VehicleCommand()
        arm_command.timestamp = self.get_clock().now().to_msg().sec
        arm_command.command = 400  # Command to arm (PX4's arm command)
        arm_command.param1 = 1.0  # Set param1 to 1 to arm the drone
        arm_command.param2 = 0.0  # Param2 is unused here
        arm_command.param3 = 0.0  # Param3 is unused here
        arm_command.param4 = 0.0  # Param4 is unused here
        arm_command.target_system = 1  # Drone 1
        arm_command.target_component = 1
        self.drone_1_publisher.publish(arm_command)

        arm_command.target_system = 2  # Drone 2
        self.drone_2_publisher.publish(arm_command)

        arm_command.target_system = 3  # Drone 3
        self.drone_3_publisher.publish(arm_command)

        # After arming, mark drones as armed
        self.drones_armed = True
        self.get_logger().info("Drones armed.")

    def set_offboard_mode(self):
        # Send the Offboard mode command to each drone
        self.get_logger().info("Setting drones to Offboard mode...")

        offboard_command = VehicleCommand()
        offboard_command.timestamp = self.get_clock().now().to_msg().sec
        offboard_command.command = 176  # Command to switch to Offboard mode
        offboard_command.param1 = 1.0  # Param1 to 1.0 to indicate Offboard mode
        offboard_command.param2 = 0.0  # Param2 can be used for additional options, left as 0
        offboard_command.param3 = 0.0  # Param3 can be used for additional options, left as 0
        offboard_command.param4 = 0.0  # Param4 can be used for additional options, left as 0
        offboard_command.target_system = 1  # Drone 1
        offboard_command.target_component = 1
        self.drone_1_publisher.publish(offboard_command)

        offboard_command.target_system = 2  # Drone 2
        self.drone_2_publisher.publish(offboard_command)

        offboard_command.target_system = 3  # Drone 3
        self.drone_3_publisher.publish(offboard_command)

        # After setting Offboard mode, mark drones as in Offboard mode
        self.drones_in_offboard_mode = True
        self.get_logger().info("Drones set to Offboard mode.")

    def send_altitude_command(self):
        # Create vehicle command messages for each drone to set the altitude
        vehicle_command_msg_1 = self.create_vehicle_command(self.target_altitude_drone_1, 1)
        vehicle_command_msg_2 = self.create_vehicle_command(self.target_altitude_drone_2, 2)
        vehicle_command_msg_3 = self.create_vehicle_command(self.target_altitude_drone_3, 3)

        # Publish the altitude commands to each drone
        self.drone_1_publisher.publish(vehicle_command_msg_1)
        self.drone_2_publisher.publish(vehicle_command_msg_2)
        self.drone_3_publisher.publish(vehicle_command_msg_3)

        # Log the altitude commands
        self.get_logger().info(f"Sent altitude command to Drone 1: {self.target_altitude_drone_1} meters")
        self.get_logger().info(f"Sent altitude command to Drone 2: {self.target_altitude_drone_2} meters")
        self.get_logger().info(f"Sent altitude command to Drone 3: {self.target_altitude_drone_3} meters")

    def create_vehicle_command(self, target_altitude, drone_id):
        # Create a vehicle command message to set the altitude for a specific drone
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.get_clock().now().to_msg().sec
        vehicle_command_msg.param1 = 0.0  # No specific value for param1 in altitude
        vehicle_command_msg.param2 = target_altitude
        # vehicle_command_msg.param3 = 0
        # vehicle_command_msg.param4 = 0
        vehicle_command_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE  # Command to set altitude (this could vary, check PX4 documentation)
        vehicle_command_msg.target_system = drone_id  # Target system for each drone
        vehicle_command_msg.target_component = 1

        return vehicle_command_msg

def main(args=None):
    rclpy.init(args=args)
    node = DroneAltitudeControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''


'''import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand  # Adjust this import if a different message type is used
from std_msgs.msg import Header

class DroneAltitudeControl(Node):

    def __init__(self):
        super().__init__('drone_altitude_control')
        
        # Create publishers for each drone's vehicle_command topic
        self.drone_1_publisher = self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command', 10)
        self.drone_2_publisher = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10)
        self.drone_3_publisher = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10)

        # Set different target altitudes for each drone (in meters)
        self.target_altitude_drone_1 = 20.0  # Drone 1 Altitude
        self.target_altitude_drone_2 = 50.0   # Drone 2 Altitude
        self.target_altitude_drone_3 = 80.0  # Drone 3 Altitude

        # Set up a timer to send the altitude command
        self.timer = self.create_timer(1.0, self.send_altitude_command)  # Send command every second

    def send_altitude_command(self):
        # Create vehicle command messages for each drone to set the altitude
        vehicle_command_msg_1 = self.create_vehicle_command(self.target_altitude_drone_1, 1)
        vehicle_command_msg_2 = self.create_vehicle_command(self.target_altitude_drone_2, 2)
        vehicle_command_msg_3 = self.create_vehicle_command(self.target_altitude_drone_3, 3)

        # Publish the altitude commands to each drone
        self.drone_1_publisher.publish(vehicle_command_msg_1)
        self.drone_2_publisher.publish(vehicle_command_msg_2)
        self.drone_3_publisher.publish(vehicle_command_msg_3)

        # Log the altitude commands
        self.get_logger().info(f"Sent altitude command to Drone 1: {self.target_altitude_drone_1} meters")
        self.get_logger().info(f"Sent altitude command to Drone 2: {self.target_altitude_drone_2} meters")
        self.get_logger().info(f"Sent altitude command to Drone 3: {self.target_altitude_drone_3} meters")

    def create_vehicle_command(self, target_altitude, drone_id):
        # Create a vehicle command message to set the altitude for a specific drone
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.get_clock().now().to_msg().sec
        vehicle_command_msg.param1 = 0.0  # No specific value for param1 in altitude
        vehicle_command_msg.param2 = target_altitude
        # vehicle_command_msg.param3 = 0
        # vehicle_command_msg.param4 = 0
        vehicle_command_msg.command = VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE  # Command to set altitude (this could vary, check PX4 documentation)
        vehicle_command_msg.target_system = drone_id  # Target system for each drone
        vehicle_command_msg.target_component = 1

        return vehicle_command_msg

def main(args=None):
    rclpy.init(args=args)
    node = DroneAltitudeControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''


'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import time

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.offboard_control_publisher = self.create_publisher(OffboardControlMode, '/px4_0/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/px4_0/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command', 10)
        
        # Subscriber
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/px4_0/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleControlMode()
        self.takeoff_altitude = 20.0  # meters
        self.hover_time = 120.0  # seconds
        self.start_hover_time = None
        self.state = 'INIT'

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_callback(self):
        if self.state == 'INIT':
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            
            if self.offboard_setpoint_counter == 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.arm()
            
            if self.offboard_setpoint_counter > 10 and self.vehicle_status.flag_armed and self.vehicle_status.flag_control_offboard_enabled:
                self.state = 'TAKEOFF'
                self.get_logger().info('Taking off')
            
            self.offboard_setpoint_counter += 1

        elif self.state == 'TAKEOFF':
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -self.takeoff_altitude)
            
            if self.vehicle_status.flag_control_position_enabled:
                self.state = 'HOVER'
                self.start_hover_time = time.time()
                self.get_logger().info('Reached hover altitude, starting hover')

        elif self.state == 'HOVER':
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -self.takeoff_altitude)
            
            if time.time() - self.start_hover_time > self.hover_time:
                self.state = 'LAND'
                self.get_logger().info('Hover complete, initiating landing')

        elif self.state == 'LAND':
            self.land()
            self.state = 'IDLE'
            self.get_logger().info('Landing command sent')

        elif self.state == 'IDLE':
            pass

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import time
import math

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Drone-specific parameters
        self.drones = {
            'px4_0': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_0/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_0/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_0/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_0'), qos_profile
                ),
                'target_altitude': 50.0,  # Half of px4_1's height
                'trajectory_type': 'right_curve'
            },
            'px4_1': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_1/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_1'), qos_profile
                ),
                'target_altitude': 100.0,  # Above the building
                'trajectory_type': 'forward'
            },
            'px4_2': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_2/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_2'), qos_profile
                ),
                'target_altitude': 50.0,  # Half of px4_1's height
                'trajectory_type': 'left_curve'
            }
        }
        
        # Timer for control
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = {drone: VehicleControlMode() for drone in self.drones}
        self.state = {drone: 'INIT' for drone in self.drones}
        self.start_hover_time = {drone: None for drone in self.drones}
        
        # Staggered launch times
        self.launch_delay = {
            'px4_0': 10.0,   # Last to launch
            'px4_1': 2.0,    # First to launch
            'px4_2': 7.0     # Second to launch
        }
        self.launch_start_time = time.time()

    def vehicle_control_mode_callback(self, msg, drone_id):
        self.vehicle_status[drone_id] = msg

    def publish_offboard_control_mode(self, drone_id):
        drone = self.drones[drone_id]
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['offboard_control_publisher'].publish(msg)

    def publish_trajectory_setpoint(self, drone_id, x, y, z, time_elapsed):
        drone = self.drones[drone_id]
        msg = TrajectorySetpoint()
        
        # Different trajectory based on drone type
        if drone['trajectory_type'] == 'forward':
            # Simple forward movement
            msg.position = [x + time_elapsed, y, z]
        elif drone['trajectory_type'] == 'right_curve':
            # Curved trajectory to the right
            msg.position = [
                x + time_elapsed, 
                y + math.sin(time_elapsed * 0.5) * 2, 
                z
            ]
        elif drone['trajectory_type'] == 'left_curve':
            # Curved trajectory to the left
            msg.position = [
                x + time_elapsed, 
                y - math.sin(time_elapsed * 0.5) * 2, 
                z
            ]
        
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['trajectory_setpoint_publisher'].publish(msg)

    def publish_vehicle_command(self, drone_id, command, param1=0.0, param2=0.0):
        drone = self.drones[drone_id]
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = int(drone_id.split('_')[1]) + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['vehicle_command_publisher'].publish(msg)

    def arm(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_callback(self):
        current_time = time.time()
        time_since_start = current_time - self.launch_start_time

        for drone_id in self.drones:
            # Check if it's time to start this drone
            if time_since_start < self.launch_delay[drone_id]:
                continue

            drone = self.drones[drone_id]
            target_altitude = -drone['target_altitude']  # Negative for NED frame

            if self.state[drone_id] == 'INIT':
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, 0.0, 0)
                
                if self.offboard_setpoint_counter == 10:
                    self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.arm(drone_id)
                
                if (self.offboard_setpoint_counter > 10 and 
                    self.vehicle_status[drone_id].flag_armed and 
                    self.vehicle_status[drone_id].flag_control_offboard_enabled):
                    self.state[drone_id] = 'TAKEOFF'
                    self.get_logger().info(f'Taking off {drone_id}')
                
                self.offboard_setpoint_counter += 1

            elif self.state[drone_id] == 'TAKEOFF':
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, target_altitude, 0)
                
                if self.vehicle_status[drone_id].flag_control_position_enabled:
                    self.state[drone_id] = 'FORWARD'
                    self.start_hover_time[drone_id] = time.time()
                    self.get_logger().info(f'{drone_id} reached hover altitude, starting forward movement')

            elif self.state[drone_id] == 'FORWARD':
                time_elapsed = time.time() - self.start_hover_time[drone_id]
                self.publish_offboard_control_mode(drone_id)
                
                # Different forward movements based on drone
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, target_altitude, time_elapsed)
                
                # Stop after 30 seconds of forward movement
                if time_elapsed > 30:
                    self.state[drone_id] = 'LAND'
                    self.get_logger().info(f'{drone_id} forward movement complete')

            elif self.state[drone_id] == 'LAND':
                self.land(drone_id)
                self.state[drone_id] = 'IDLE'
                self.get_logger().info(f'{drone_id} landing')

            elif self.state[drone_id] == 'IDLE':
                pass

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    rclpy.spin(multi_drone_control)
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''


'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import time
import math

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Drone-specific parameters
        self.drones = {
            'px4_0': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_0/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_0/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_0/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_0/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_0'), qos_profile
                ),
                'target_altitude': 50.0,
                'trajectory_type': 'right_curve',
                'current_position': [0.0, 0.0, 0.0]
            },
            'px4_1': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_1/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_1'), qos_profile
                ),
                'target_altitude': 100.0,
                'trajectory_type': 'forward',
                'current_position': [0.0, 0.0, 0.0]
            },
            'px4_2': {
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, '/px4_2/fmu/out/vehicle_control_mode', 
                    lambda msg: self.vehicle_control_mode_callback(msg, 'px4_2'), qos_profile
                ),
                'target_altitude': 50.0,
                'trajectory_type': 'left_curve',
                'current_position': [0.0, 0.0, 0.0]
            }
        }
        
        # Timer for control
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = {drone: VehicleControlMode() for drone in self.drones}
        self.state = {drone: 'INIT' for drone in self.drones}
        self.start_mission_time = None
        self.mission_duration = 40.0  # Total mission duration
        
        # Tracking mission-wide status
        self.all_drones_ready = False
        self.mission_started = False

    def vehicle_control_mode_callback(self, msg, drone_id):
        self.vehicle_status[drone_id] = msg
        
        # Extract current position (assuming this is available in the control mode message)
        # Note: You might need to adjust this based on actual PX4 message structure
        self.drones[drone_id]['current_position'] = [
            msg.x,  # Replace with actual x position extraction
            msg.y,  # Replace with actual y position extraction
            msg.z   # Replace with actual z position extraction
        ]

    def publish_offboard_control_mode(self, drone_id):
        drone = self.drones[drone_id]
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['offboard_control_publisher'].publish(msg)

    def publish_trajectory_setpoint(self, drone_id, time_elapsed):
        drone = self.drones[drone_id]
        msg = TrajectorySetpoint()
        target_altitude = -drone['target_altitude']
        
        # Different trajectory based on drone type
        if drone['trajectory_type'] == 'forward':
            # Simple forward movement
            msg.position = [0.0, time_elapsed, target_altitude]
        elif drone['trajectory_type'] == 'right_curve':
            # Curved trajectory to the right
            msg.position = [
                math.sin(time_elapsed * 0.5) * 2,
                float(time_elapsed),
                target_altitude
            ]
        elif drone['trajectory_type'] == 'left_curve':
            # Curved trajectory to the left
            msg.position = [
                -math.sin(time_elapsed * 0.5) * 2,
                float(time_elapsed),
                target_altitude
            ]
        
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['trajectory_setpoint_publisher'].publish(msg)

    def publish_vehicle_command(self, drone_id, command, param1=0.0, param2=0.0):
        drone = self.drones[drone_id]
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = int(drone_id.split('_')[1]) + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['vehicle_command_publisher'].publish(msg)

    def arm(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def land(self, drone_id):
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def check_and_handle_landing(self, drone_id):
        drone = self.drones[drone_id]
        current_z = drone['current_position'][2]
        
        # Check if drone is very close to ground (absolute value of z < 0.1)
        if abs(current_z) < 0.1:
            # Disarm and release offboard mode
            self.disarm(drone_id)
            self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 0.0)  # Return to manual mode
            self.get_logger().info(f'{drone_id} disarmed and offboard mode released')
            return True
        return False

    def timer_callback(self):
        # Check if all drones are ready to start simultaneously
        if not self.all_drones_ready:
            ready_count = sum(
                self.vehicle_status[drone].flag_armed and 
                self.vehicle_status[drone].flag_control_offboard_enabled 
                for drone in self.drones
            )
            
            if ready_count == len(self.drones):
                self.all_drones_ready = True
                self.start_mission_time = time.time()
                self.mission_started = True
                self.get_logger().info('All drones ready. Starting simultaneous mission.')

        # If mission has started
        if self.mission_started:
            time_since_start = time.time() - self.start_mission_time

            # Mission progression
            if time_since_start < self.mission_duration:
                # Simultaneous forward movement
                for drone_id in self.drones:
                    self.publish_offboard_control_mode(drone_id)
                    self.publish_trajectory_setpoint(drone_id, time_since_start)
            else:
                # Landing phase
                for drone_id in self.drones:
                    # Send land command
                    self.land(drone_id)
                    
                    # Check if drone has landed and needs to be disarmed
                    self.check_and_handle_landing(drone_id)

        # Initial setup phase
        else:
            for drone_id in self.drones:
                # Offboard control mode setup
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0)
                
                # Arm and set offboard mode
                if self.offboard_setpoint_counter == 10:
                    self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.arm(drone_id)
            
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    rclpy.spin(multi_drone_control)
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''

'''import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode, VehicleLocalPosition
import time

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Numpy-based waypoint generation
        self.drones = {
            'px4_0': self.generate_trajectory_waypoints(
                trajectory_type='forward', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=0.0
            ),
            'px4_1': self.generate_trajectory_waypoints(
                trajectory_type='right_curve', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=10.0
            ),
            'px4_2': self.generate_trajectory_waypoints(
                trajectory_type='left_curve', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=-10.0
            )
        }
        
        # ROS2 publishers and subscribers
        for drone_id, drone_params in self.drones.items():
            drone_params.update({
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, f'/{drone_id}/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, f'/{drone_id}/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, f'/{drone_id}/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, f'/{drone_id}/fmu/out/vehicle_control_mode', 
                    lambda msg, did=drone_id: self.vehicle_control_mode_callback(msg, did), 
                    qos_profile
                ),
                'local_position_subscriber': self.create_subscription(
                    VehicleLocalPosition, f'/{drone_id}/fmu/out/vehicle_local_position',
                    lambda msg, did=drone_id: self.local_position_callback(msg, did),
                    qos_profile
                ),
                'current_position': [0.0, 0.0, 0.0],
                'current_waypoint_index': 0,
                'state': 'INIT',
                'is_armed': False,
                'is_offboard': False
            })
        
        # Mission control variables
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.vehicle_status = {drone: VehicleControlMode() for drone in self.drones}
        self.all_drones_ready = False
        self.mission_started = False
        self.mission_duration = 60.0
        self.start_mission_time = None

    def local_position_callback(self, msg, drone_id):
        """
        Update current position from VehicleLocalPosition message
        """
        self.drones[drone_id]['current_position'] = [
            msg.x, msg.y, msg.z
        ]

    def vehicle_control_mode_callback(self, msg, drone_id):
        """
        Update vehicle status and control mode
        """
        self.vehicle_status[drone_id] = msg
        drone = self.drones[drone_id]
        
        # Update armed and offboard status
        drone['is_armed'] = msg.flag_armed
        drone['is_offboard'] = msg.flag_control_offboard_enabled

    def generate_trajectory_waypoints(self, trajectory_type, total_distance, height, horizontal_displacement):
        """
        Generate waypoints for different trajectory types using NumPy
        """
        # Number of waypoints
        num_waypoints = 20
        
        # Initialize base waypoints
        waypoints = np.zeros((num_waypoints, 3))
        
        # Common vertical progression (height)
        waypoints[:, 2] = -height
        
        # Horizontal and forward progression based on trajectory type
        if trajectory_type == 'forward':
            # Straight line forward
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
        
        elif trajectory_type == 'right_curve':
            # Curved trajectory to the right
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
            waypoints[:, 1] = np.linspace(0, horizontal_displacement, num_waypoints)
            
            # Apply smooth curve using sine function
            waypoints[:, 1] *= np.sin(np.linspace(0, np.pi/2, num_waypoints))
        
        elif trajectory_type == 'left_curve':
            # Curved trajectory to the left
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
            waypoints[:, 1] = np.linspace(0, horizontal_displacement, num_waypoints)
            
            # Apply smooth curve using sine function
            waypoints[:, 1] *= np.sin(np.linspace(0, np.pi/2, num_waypoints))
        
        return {
            'waypoints': waypoints,
            'trajectory_type': trajectory_type,
            'total_distance': total_distance,
            'height': height,
            'horizontal_displacement': horizontal_displacement
        }

    def is_waypoint_reached(self, current_pos, target_pos, tolerance=0.5):
        """
        Check if current position is close enough to target waypoint
        """
        return np.linalg.norm(np.array(current_pos) - np.array(target_pos)) < tolerance

    def publish_offboard_control_mode(self, drone_id):
        """
        Publish offboard control mode
        """
        drone = self.drones[drone_id]
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['offboard_control_publisher'].publish(msg)

    def publish_trajectory_setpoint(self, drone_id):
        """
        Publish trajectory setpoint based on current waypoint
        """
        drone = self.drones[drone_id]
        current_index = drone['current_waypoint_index']
        waypoints = drone['waypoints']
        
        # Get current waypoint
        current_waypoint = waypoints[current_index]
        
        # Check if waypoint is reached
        if self.is_waypoint_reached(drone['current_position'], current_waypoint):
            # Move to next waypoint
            current_index = min(current_index + 1, len(waypoints) - 1)
            drone['current_waypoint_index'] = current_index
        
        # Prepare trajectory setpoint message
        msg = TrajectorySetpoint()
        msg.position = current_waypoint.tolist()
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Publish trajectory setpoint
        drone['trajectory_setpoint_publisher'].publish(msg)

    def publish_vehicle_command(self, drone_id, command, param1=0.0, param2=0.0):
        """
        Publish vehicle command
        """
        drone = self.drones[drone_id]
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = int(drone_id.split('_')[1]) + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['vehicle_command_publisher'].publish(msg)

    def arm(self, drone_id):
        """
        Arm the drone
        """
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self, drone_id):
        """
        Disarm the drone
        """
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def land(self, drone_id):
        """
        Send land command
        """
        self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def check_and_handle_landing(self, drone_id):
        """
        Check if drone has landed and handle disarming
        """
        drone = self.drones[drone_id]
        current_z = drone['current_position'][2]
        
        # Check if drone is very close to ground
        if abs(current_z) < 0.1:
            # Disarm and release offboard mode
            self.disarm(drone_id)
            self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 0.0)
            self.get_logger().info(f'{drone_id} disarmed and offboard mode released')
            return True
        return False

    def timer_callback(self):
        """
        Main mission control callback
        """
        # Check if all drones are ready to start simultaneously
        # Check if all drones have received a vehicle status
        if not all(status is not None and isinstance(status, VehicleControlMode) for status in self.vehicle_status.values()):
            self.get_logger().info('Waiting for all drone statuses...')
            return

        # Count drones that are armed
        ready_count = sum(
            1 for drone in self.drones 
            if self.vehicle_status[drone].flag_armed
        )
            
        if ready_count == len(self.drones):
            self.all_drones_ready = True
            self.start_mission_time = time.time()
            self.mission_started = True
            self.get_logger().info('All drones ready. Starting simultaneous mission.')

        # If mission has started
        if self.mission_started:
            time_since_start = time.time() - self.start_mission_time

            # Mission progression
            if time_since_start < self.mission_duration:
                # Simultaneous movement
                for drone_id in self.drones:
                    self.publish_offboard_control_mode(drone_id)
                    self.publish_trajectory_setpoint(drone_id)
            else:
                # Landing phase
                for drone_id in self.drones:
                    # Send land command
                    self.land(drone_id)
                    
                    # Check if drone has landed and needs to be disarmed
                    self.check_and_handle_landing(drone_id)

        # Initial setup phase
        else:
            for drone_id in self.drones:
                # Offboard control mode setup
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id)
                
                # Arm and set offboard mode
                if self.offboard_setpoint_counter == 10:
                    self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.arm(drone_id)
            
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    rclpy.spin(multi_drone_control)
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode, VehicleLocalPosition
import time

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Increased depth to help with payload issues
        )
        
        # Numpy-based waypoint generation
        self.drones = {
            'px4_0': self.generate_trajectory_waypoints(
                trajectory_type='forward', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=0.0
            ),
            'px4_1': self.generate_trajectory_waypoints(
                trajectory_type='right_curve', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=10.0
            ),
            'px4_2': self.generate_trajectory_waypoints(
                trajectory_type='left_curve', 
                total_distance=20.0, 
                height=5.0,
                horizontal_displacement=-10.0
            )
        }
        
        # ROS2 publishers and subscribers
        for drone_id, drone_params in self.drones.items():
            drone_params.update({
                'offboard_control_publisher': self.create_publisher(OffboardControlMode, f'/{drone_id}/fmu/in/offboard_control_mode', 10),
                'trajectory_setpoint_publisher': self.create_publisher(TrajectorySetpoint, f'/{drone_id}/fmu/in/trajectory_setpoint', 10),
                'vehicle_command_publisher': self.create_publisher(VehicleCommand, f'/{drone_id}/fmu/in/vehicle_command', 10),
                'vehicle_control_mode_subscriber': self.create_subscription(
                    VehicleControlMode, f'/{drone_id}/fmu/out/vehicle_control_mode', 
                    lambda msg, did=drone_id: self.vehicle_control_mode_callback(msg, did), 
                    qos_profile
                ),
                'local_position_subscriber': self.create_subscription(
                    VehicleLocalPosition, f'/{drone_id}/fmu/out/vehicle_local_position',
                    lambda msg, did=drone_id: self.local_position_callback(msg, did),
                    qos_profile
                ),
                'current_position': [0.0, 0.0, 0.0],
                'current_waypoint_index': 0,
                'state': 'INIT',
                'arm_requested': False,
                'offboard_requested': False,
                'arm_attempt_counter': 0,
                'offboard_attempt_counter': 0
            })
        
        # Mission control variables
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.vehicle_status = {drone: None for drone in self.drones}
        self.all_drones_ready = False
        self.mission_started = False
        self.mission_duration = 60.0
        self.start_mission_time = None
        self.takeoff_altitude = 5.0  # meters
        self.hover_time = 120.0  # seconds

    def local_position_callback(self, msg, drone_id):
        """
        Update current position from VehicleLocalPosition message
        """
        self.drones[drone_id]['current_position'] = [
            msg.x, msg.y, msg.z
        ]

    def vehicle_control_mode_callback(self, msg, drone_id):
        """
        Update vehicle status and control mode
        """
        self.vehicle_status[drone_id] = msg

    def publish_offboard_control_mode(self, drone_id):
        """
        Publish offboard control mode for a specific drone
        """
        drone = self.drones[drone_id]
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['offboard_control_publisher'].publish(msg)

    def publish_trajectory_setpoint(self, drone_id, x, y, z):
        """
        Publish trajectory setpoint for a specific drone
        """
        drone = self.drones[drone_id]
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['trajectory_setpoint_publisher'].publish(msg)

    def publish_vehicle_command(self, drone_id, command, param1=0.0, param2=0.0):
        """
        Publish vehicle command for a specific drone
        """
        drone = self.drones[drone_id]
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone['vehicle_command_publisher'].publish(msg)

    def request_arm(self, drone_id):
        """
        Request arming for a specific drone
        """
        drone = self.drones[drone_id]
        if not drone['arm_requested']:
            self.get_logger().info(f'Requesting arm for {drone_id}')
            self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            drone['arm_requested'] = True
            drone['arm_attempt_counter'] += 1

    def request_offboard(self, drone_id):
        """
        Request offboard mode for a specific drone
        """
        drone = self.drones[drone_id]
        if not drone['offboard_requested']:
            self.get_logger().info(f'Requesting offboard mode for {drone_id}')
            self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            drone['offboard_requested'] = True
            drone['offboard_attempt_counter'] += 1

    def is_armed_and_offboard(self, drone_id):
        """
        Check if a specific drone is armed and in offboard mode
        """
        status = self.vehicle_status[drone_id]
        if status is None:
            return False
        
        return (status.flag_armed and 
                status.flag_control_offboard_enabled)

    def generate_trajectory_waypoints(self, trajectory_type, total_distance, height, horizontal_displacement):
        """
        Generate waypoints for different trajectory types using NumPy
        """
        # Number of waypoints
        num_waypoints = 20
        
        # Initialize base waypoints
        waypoints = np.zeros((num_waypoints, 3))
        
        # Common vertical progression (height)
        waypoints[:, 2] = -height
        
        # Horizontal and forward progression based on trajectory type
        if trajectory_type == 'forward':
            # Straight line forward
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
        
        elif trajectory_type == 'right_curve':
            # Curved trajectory to the right
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
            waypoints[:, 1] = np.linspace(0, horizontal_displacement, num_waypoints)
            
            # Apply smooth curve using sine function
            waypoints[:, 1] *= np.sin(np.linspace(0, np.pi/2, num_waypoints))
        
        elif trajectory_type == 'left_curve':
            # Curved trajectory to the left
            waypoints[:, 0] = np.linspace(0, total_distance, num_waypoints)
            waypoints[:, 1] = np.linspace(0, horizontal_displacement, num_waypoints)
            
            # Apply smooth curve using sine function
            waypoints[:, 1] *= np.sin(np.linspace(0, np.pi/2, num_waypoints))
        
        return {
            'waypoints': waypoints,
            'trajectory_type': trajectory_type,
            'total_distance': total_distance,
            'height': height,
            'horizontal_displacement': horizontal_displacement
        }

    def timer_callback(self):
        # Check if all vehicle statuses are available
        if not all(status is not None for status in self.vehicle_status.values()):
            return

        # Process each drone
        for drone_id, drone in self.drones.items():
            # Ensure we have a valid vehicle status
            status = self.vehicle_status[drone_id]

            if drone['state'] == 'INIT':
                # Continuously publish zero setpoint and offboard control mode
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, 0.0)

                # Attempt to arm if not already armed
                if not status.flag_armed:
                    if drone['arm_attempt_counter'] < 10:
                        self.request_arm(drone_id)
                    else:
                        self.get_logger().error(f'Failed to arm {drone_id} after multiple attempts')
                        drone['state'] = 'ERROR'
                        continue

                # Attempt to enter offboard mode
                if (status.flag_armed and 
                    not status.flag_control_offboard_enabled):
                    if drone['offboard_attempt_counter'] < 10:
                        self.request_offboard(drone_id)
                    else:
                        self.get_logger().error(f'Failed to enter offboard mode for {drone_id} after multiple attempts')
                        drone['state'] = 'ERROR'
                        continue

                # Check if fully armed and in offboard mode
                if self.is_armed_and_offboard(drone_id):
                    drone['state'] = 'TAKEOFF'
                    self.get_logger().info(f'{drone_id} armed and in offboard mode, initiating takeoff')

            elif drone['state'] == 'TAKEOFF':
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, -self.takeoff_altitude)
                
                if status.flag_control_position_enabled:
                    drone['state'] = 'HOVER'
                    drone['start_hover_time'] = time.time()
                    self.get_logger().info(f'{drone_id} reached hover altitude, starting hover')

            elif drone['state'] == 'HOVER':
                self.publish_offboard_control_mode(drone_id)
                self.publish_trajectory_setpoint(drone_id, 0.0, 0.0, -self.takeoff_altitude)
                
                if time.time() - drone.get('start_hover_time', 0) > self.hover_time:
                    drone['state'] = 'LAND'
                    self.get_logger().info(f'{drone_id} hover complete, initiating landing')

            elif drone['state'] == 'LAND':
                # Use vehicle command to initiate landing
                self.publish_vehicle_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                drone['state'] = 'IDLE'
                self.get_logger().info(f'{drone_id} landing command sent')

            elif drone['state'] == 'ERROR':
                # Optional: Add error handling or recovery logic
                pass

            elif drone['state'] == 'IDLE':
                pass

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    rclpy.spin(multi_drone_control)
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()