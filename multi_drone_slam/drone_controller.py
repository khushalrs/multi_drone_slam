'''import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GimbalPitchController(Node):
    def __init__(self):
        super().__init__('gimbal_pitch_controller')
        self.publisher = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_pitch', 10)
        self.timer = self.create_timer(0.01, self.publish_pitch)  # Publish every 100ms
        self.pitch_value = 30.0  # Desired pitch angle

    def publish_pitch(self):
        msg = Float64()
        msg.data = self.pitch_value
        self.publisher.publish(msg)
        self.get_logger().info(f"Published gimbal pitch: {self.pitch_value}")

def main(args=None):
    rclpy.init(args=args)
    node = GimbalPitchController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleCommand

class PX4ParameterSetter(Node):
    def __init__(self):
        super().__init__('px4_parameter_setter')
        self.publisher1 = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', 10)
        self.timer = self.create_timer(1.0, self.set_parameter)  # Run every second
        self.parameter_set = False  # Flag to ensure we only send once

        self.publisher2 = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_pitch', 10)
        self.timer = self.create_timer(0.01, self.publish_pitch)  # Publish every 100ms
        self.pitch_value = 30.0  # Desired pitch angle

    def publish_pitch(self):
        msg = Float64()
        msg.data = self.pitch_value
        self.publisher2.publish(msg)
        self.get_logger().info(f"Published gimbal pitch: {self.pitch_value}")

    def set_parameter(self):
        if not self.parameter_set:
            # Create VehicleCommand message
            msg = VehicleCommand()
            msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert nanoseconds to microseconds
            msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER  # Command ID for setting parameters
            
            # Example: Setting MNT_MODE_IN (numeric ID: 982) to MAVLink control (value: 4)
            msg.param1 = 982.0  # Numeric ID for MNT_MODE_IN
            msg.param2 = 4.0  # Value for MAVLink control
            msg.target_system = 1  # Target system (usually 1 for SITL)
            msg.target_component = 1  # Target component (usually 1 for SITL)

            # Publish the command
            self.publisher1.publish(msg)
            self.get_logger().info("Sent parameter set command: MNT_MODE_IN -> MAVLink control (value: 4)")
            
            # Ensure we only send this once
            self.parameter_set = True

def main(args=None):
    rclpy.init(args=args)
    node = PX4ParameterSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()'''


'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import time

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Drone configurations
        self.drones = {
            'px4_0': {
                'namespace': '/px4_0',
                'initial_position': [0.0, 0.0, 0.0],
                'target_position': [0.0, 30.0, -20.0]
            },
            'px4_1': {
                'namespace': '/px4_1', 
                'initial_position': [-10.0, 0.0, 0.0],
                'target_position': [-10.0, 30.0, -20.0]
            },
            'px4_2': {
                'namespace': '/px4_2',
                'initial_position': [10.0, 0.0, 0.0], 
                'target_position': [10.0, 30.0, -20.0]
            }
        }
        
        # Global state tracking
        self.global_state = 'INIT'
        self.offboard_setpoint_counter = 0
        
        # Initialize my_pub dictionary
        self.my_pub = {}
        self.vehicle_statuses = {}
        
        # Dynamically create my_pub for each drone
        for drone_name, drone_config in self.drones.items():
            # Create a nested dictionary for each drone's my_pub
            self.my_pub[drone_name] = {
                'offboard_control': self.create_publisher(
                    OffboardControlMode, 
                    f'{drone_config["namespace"]}/fmu/in/offboard_control_mode', 
                    10
                ),
                'trajectory_setpoint': self.create_publisher(
                    TrajectorySetpoint, 
                    f'{drone_config["namespace"]}/fmu/in/trajectory_setpoint', 
                    10
                ),
                'vehicle_command': self.create_publisher(
                    VehicleCommand, 
                    f'{drone_config["namespace"]}/fmu/in/vehicle_command', 
                    10
                )
            }
            
            # Create vehicle status dictionary for each drone
            self.vehicle_statuses[drone_name] = {
                'status': VehicleControlMode(),
                'ready_for_next_state': False
            }
            
            # Subscribe to vehicle control mode for each drone
            self.create_subscription(
                VehicleControlMode, 
                f'{drone_config["namespace"]}/fmu/out/vehicle_control_mode', 
                lambda msg, drone=drone_name: self.vehicle_control_mode_callback(msg, drone), 
                self.qos_profile
            )
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Hover and takeoff parameters
        self.takeoff_altitude = 20.0  # meters
        self.ground_level_threshold = 0.1  # meters above ground

    def vehicle_control_mode_callback(self, msg, drone_name):
        self.vehicle_statuses[drone_name]['status'] = msg

    def publish_offboard_control_mode(self, drone_name):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.my_pub[drone_name]['offboard_control'].publish(msg)

    def publish_trajectory_setpoint(self, drone_name, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.my_pub[drone_name]['trajectory_setpoint'].publish(msg)

    def publish_vehicle_command(self, drone_name, command, param1=0.0, param2=0.0):
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
        self.my_pub[drone_name]['vehicle_command'].publish(msg)

    def timer_callback(self):
        # Global state machine
        if self.global_state == 'INIT':
            # Publish initial setpoints
            for drone_name, drone_config in self.drones.items():
                self.publish_offboard_control_mode(drone_name)
                initial_pos = drone_config['initial_position']
                self.publish_trajectory_setpoint(drone_name, initial_pos[0], initial_pos[1], initial_pos[2])
            
            # Arm and set offboard mode after some setpoints
            if self.offboard_setpoint_counter == 10:
                for drone_name in self.drones:
                    self.publish_vehicle_command(drone_name, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.publish_vehicle_command(drone_name, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            # Check if all drones are armed and in offboard mode
            if self.offboard_setpoint_counter > 10:
                all_ready = all(
                    status['status'].flag_armed and 
                    status['status'].flag_control_offboard_enabled 
                    for status in self.vehicle_statuses.values()
                )
                
                if all_ready:
                    self.global_state = 'TAKEOFF'
                    self.get_logger().info('All drones ready for takeoff')
            
            self.offboard_setpoint_counter += 1

        elif self.global_state == 'TAKEOFF':
            # Simultaneous takeoff
            for drone_name, drone_config in self.drones.items():
                self.publish_offboard_control_mode(drone_name)
                self.publish_trajectory_setpoint(
                    drone_name, 
                    drone_config['initial_position'][0], 
                    drone_config['initial_position'][1], 
                    -self.takeoff_altitude
                )
            
            # Check if all drones have reached takeoff altitude
            all_at_altitude = all(
                status['status'].flag_control_position_enabled 
                for status in self.vehicle_statuses.values()
            )
            
            if all_at_altitude:
                self.global_state = 'MOVE'
                self.get_logger().info('All drones reached takeoff altitude')

        elif self.global_state == 'MOVE':
            # Simultaneous movement to target positions
            for drone_name, drone_config in self.drones.items():
                target_pos = drone_config['target_position']
                self.publish_offboard_control_mode(drone_name)
                self.publish_trajectory_setpoint(drone_name, target_pos[0], target_pos[1], target_pos[2])
            
            # Check if all drones are close to their target positions
            all_at_target = all(
                abs(drone_config['target_position'][0] - drone_config['initial_position'][0]) < 1.0 and
                abs(drone_config['target_position'][1] - drone_config['initial_position'][1]) < 1.0
                for drone_name, drone_config in self.drones.items()
            )
            
            if all_at_target:
                self.global_state = 'LAND'
                self.get_logger().info('All drones reached target positions')

        elif self.global_state == 'LAND':
            # Simultaneous landing
            for drone_name in self.drones:
                self.publish_vehicle_command(drone_name, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            self.global_state = 'DESCENDING'
            self.get_logger().info('All drones landing')

        elif self.global_state == 'DESCENDING':
            # Check if all drones are close to ground
            all_descended = all(
                abs(0.0) <= self.ground_level_threshold
                for drone_name in self.drones
            )
            
            if all_descended:
                # Disarm and disable offboard mode for all drones
                for drone_name in self.drones:
                    self.publish_vehicle_command(drone_name, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 1.0)
                    self.publish_vehicle_command(drone_name, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                
                self.global_state = 'IDLE'
                self.get_logger().info('All drones disarmed')

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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait

class DronController:
    def __init__(self, node, drone_name, namespace, initial_position, target_position):
        self.node = node
        self.drone_name = drone_name
        self.namespace = namespace
        self.initial_position = initial_position
        self.target_position = target_position
        
        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.offboard_control_publisher = node.create_publisher(
            OffboardControlMode, 
            f'{namespace}/fmu/in/offboard_control_mode', 
            10
        )
        self.trajectory_setpoint_publisher = node.create_publisher(
            TrajectorySetpoint, 
            f'{namespace}/fmu/in/trajectory_setpoint', 
            10
        )
        self.vehicle_command_publisher = node.create_publisher(
            VehicleCommand, 
            f'{namespace}/fmu/in/vehicle_command', 
            10
        )
        
        # Subscriber
        self.vehicle_status = VehicleControlMode()
        node.create_subscription(
            VehicleControlMode, 
            f'{namespace}/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_callback, 
            self.qos_profile
        )
        
        # Control parameters
        self.takeoff_altitude = 20.0
        self.ground_level_threshold = 0.1
        self.state = 'INIT'
        self.offboard_setpoint_counter = 0

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_control_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
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
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def wait_for_condition(self, condition, timeout=30):
        start_time = time.time()
        while not condition():
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                self.node.get_logger().error(f'{self.drone_name}: Condition timeout')
                return False
        return True

    def run_mission(self):
        # INIT State
        self.node.get_logger().info(f'{self.drone_name}: Initializing')
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.initial_position[0], 
                self.initial_position[1], 
                self.initial_position[2]
            )
        
        # Arm and set offboard mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        # Wait for arming and offboard mode
        if not self.wait_for_condition(lambda: (
            self.vehicle_status.flag_armed and 
            self.vehicle_status.flag_control_offboard_enabled
        )):
            return
        
        # TAKEOFF State
        self.node.get_logger().info(f'{self.drone_name}: Taking off')
        for _ in range(50):  # Longer duration for stable takeoff
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.initial_position[0], 
                self.initial_position[1], 
                -self.takeoff_altitude
            )
        
        # Wait for takeoff
        if not self.wait_for_condition(lambda: 
            self.vehicle_status.flag_control_position_enabled
        ):
            return
        
        # MOVE State
        self.node.get_logger().info(f'{self.drone_name}: Moving to target')
        for _ in range(100):  # Longer duration for movement
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.target_position[0], 
                self.target_position[1], 
                self.target_position[2]
            )
        
        # LAND State
        self.node.get_logger().info(f'{self.drone_name}: Landing')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        # Wait for landing
        if not self.wait_for_condition(lambda: 
            abs(0.0) <= self.ground_level_threshold
        ):
            return
        
        # Disarm
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 1.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        
        self.node.get_logger().info(f'{self.drone_name}: Mission complete')

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # Drone configurations
        self.drones = {
            'px4_0': {
                'namespace': '/px4_0',
                'initial_position': [0.0, 0.0, 0.0],
                'target_position': [0.0, 30.0, -20.0]
            },
            'px4_1': {
                'namespace': '/px4_1', 
                'initial_position': [-10.0, 0.0, 0.0],
                'target_position': [-10.0, 30.0, -20.0]
            },
            'px4_2': {
                'namespace': '/px4_2',
                'initial_position': [10.0, 0.0, 0.0], 
                'target_position': [10.0, 30.0, -20.0]
            }
        }
        
        # Create drone controllers
        self.drone_controllers = {}
        for drone_name, config in self.drones.items():
            self.drone_controllers[drone_name] = DronController(
                self, 
                drone_name, 
                config['namespace'], 
                config['initial_position'], 
                config['target_position']
            )

    def run_missions(self):
        # Use ThreadPoolExecutor to run missions simultaneously
        with ThreadPoolExecutor(max_workers=len(self.drone_controllers)) as executor:
            # Submit missions for each drone
            future_to_drone = {
                executor.submit(controller.run_mission): drone_name 
                for drone_name, controller in self.drone_controllers.items()
            }
            
            # Wait for all missions to complete
            wait(future_to_drone)
            
            # Check for any exceptions
            for future in future_to_drone:
                try:
                    future.result()
                except Exception as exc:
                    self.get_logger().error(f'Drone mission generated an exception: {exc}')

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    
    # Run missions in a separate thread to keep ROS2 node running
    mission_thread = threading.Thread(target=multi_drone_control.run_missions)
    mission_thread.start()
    
    # Spin the node
    rclpy.spin(multi_drone_control)
    
    # Cleanup
    mission_thread.join()
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
import threading
import time
from concurrent.futures import ThreadPoolExecutor, wait, TimeoutError
from rclpy.duration import Duration

class DronController:
    def __init__(self, node, drone_name, namespace, initial_position, target_position):
        self.node = node
        self.drone_name = drone_name
        self.namespace = namespace
        self.initial_position = initial_position
        self.target_position = target_position
        
        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.offboard_control_publisher = node.create_publisher(
            OffboardControlMode, 
            f'{namespace}/fmu/in/offboard_control_mode', 
            10
        )
        self.trajectory_setpoint_publisher = node.create_publisher(
            TrajectorySetpoint, 
            f'{namespace}/fmu/in/trajectory_setpoint', 
            10
        )
        self.vehicle_command_publisher = node.create_publisher(
            VehicleCommand, 
            f'{namespace}/fmu/in/vehicle_command', 
            10
        )
        
        # Subscriber
        self.vehicle_status = VehicleControlMode()
        node.create_subscription(
            VehicleControlMode, 
            f'{namespace}/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_callback, 
            self.qos_profile
        )

        self.takeoff_altitude = 20.0
        self.ground_level_threshold = 0.1
        self.state = 'INIT'
        self.offboard_setpoint_counter = 0

        # Enhanced timeout and retry parameters
        self.max_retries = 3
        self.retry_delay = 1.0  # seconds between retries
        self.operation_timeout = 30.0  # total timeout for an operation
    
    def vehicle_control_mode_callback(self, msg):
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_control_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
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
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def wait_for_condition(self, condition, context='Operation'):
        """
        Enhanced condition waiting with retries and comprehensive logging
        
        Args:
            condition (callable): Function that returns True when condition is met
            context (str): Description of the operation being waited for
        
        Returns:
            bool: True if condition met, False otherwise
        """
        for attempt in range(self.max_retries):
            try:
                # Use ROS2 wait mechanism with a timeout
                start_time = self.node.get_clock().now()
                while not condition():
                    # Check if total timeout has been exceeded
                    if (self.node.get_clock().now() - start_time).nanoseconds / 1e9 > self.operation_timeout:
                        self.node.get_logger().error(
                            f'{self.drone_name}: {context} timeout after {self.operation_timeout} seconds'
                        )
                        return False
                    
                    # Small sleep to prevent tight looping
                    self.node.create_rate(10).sleep()
                
                # Condition successfully met
                return True
            
            except Exception as e:
                self.node.get_logger().error(
                    f'{self.drone_name}: Error in {context} (Attempt {attempt + 1}): {str(e)}'
                )
                
                # Delay before retrying
                time.sleep(self.retry_delay)
        
        # All retry attempts failed
        self.node.get_logger().error(f'{self.drone_name}: {context} failed after {self.max_retries} attempts')
        return False

    def run_mission(self):
        # The rest of the method remains largely the same, but with improved wait_for_condition calls
        # Example:
        self.node.get_logger().info(f'{self.drone_name}: Initializing')
        for _ in range(10):
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.initial_position[0], 
                self.initial_position[1], 
                self.initial_position[2]
            )
        
        # Arm and set offboard mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        # Wait for arming and offboard mode with context
        if not self.wait_for_condition(
            lambda: (
                self.vehicle_status.flag_armed and 
                self.vehicle_status.flag_control_offboard_enabled
            ),
            context='Arming and Offboard Mode'
        ):
            return
        # TAKEOFF State
        self.node.get_logger().info(f'{self.drone_name}: Taking off')
        for _ in range(50):  # Longer duration for stable takeoff
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.initial_position[0], 
                self.initial_position[1], 
                -self.takeoff_altitude
            )
        
        # Wait for takeoff
        if not self.wait_for_condition(lambda: 
            self.vehicle_status.flag_control_position_enabled
        ):
            return
        
        # MOVE State
        self.node.get_logger().info(f'{self.drone_name}: Moving to target')
        for _ in range(100):  # Longer duration for movement
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.target_position[0], 
                self.target_position[1], 
                self.target_position[2]
            )
        
        # LAND State
        self.node.get_logger().info(f'{self.drone_name}: Landing')
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        # Wait for landing
        if not self.wait_for_condition(lambda: 
            abs(0.0) <= self.ground_level_threshold
        ):
            return
        
        # Disarm
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 1.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        
        self.node.get_logger().info(f'{self.drone_name}: Mission complete')

class MultiDroneControl(Node):
    def __init__(self):
        super().__init__('multi_drone_control')
        
        # Drone configurations
        self.drones = {
            'px4_0': {
                'namespace': '/px4_0',
                'initial_position': [0.0, 0.0, 0.0],
                'target_position': [0.0, 30.0, -20.0]
            },
            'px4_1': {
                'namespace': '/px4_1', 
                'initial_position': [0.0, -3.0, 0.0],
                'target_position': [-10.0, 30.0, -20.0]
            },
            'px4_2': {
                'namespace': '/px4_2',
                'initial_position': [0.0, -6.0, 0.0], 
                'target_position': [10.0, 30.0, -20.0]
            }
        }
        
        # Create drone controllers
        self.drone_controllers = {}
        for drone_name, config in self.drones.items():
            self.drone_controllers[drone_name] = DronController(
                self, 
                drone_name, 
                config['namespace'], 
                config['initial_position'], 
                config['target_position']
            )

    def run_missions(self):
        # Use ThreadPoolExecutor to run missions simultaneously
        with ThreadPoolExecutor(max_workers=len(self.drone_controllers)) as executor:
            # Submit missions for each drone
            future_to_drone = {
                executor.submit(controller.run_mission): drone_name 
                for drone_name, controller in self.drone_controllers.items()
            }
            
            # Wait for all missions to complete
            wait(future_to_drone)
            
            # Check for any exceptions
            for future in future_to_drone:
                try:
                    future.result()
                except Exception as exc:
                    self.get_logger().error(f'Drone mission generated an exception: {exc}')

def main(args=None):
    rclpy.init(args=args)
    multi_drone_control = MultiDroneControl()
    
    # Run missions in a separate thread to keep ROS2 node running
    mission_thread = threading.Thread(target=multi_drone_control.run_missions)
    mission_thread.start()
    
    # Spin the node
    rclpy.spin(multi_drone_control)
    
    # Cleanup
    mission_thread.join()
    multi_drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



'''Edit the launch file to spawn 3 'gz_x500_gimbal' drones in gz-sim using the px4-autopilot. I am using the following command to spawn multiple drones in a single gz window:
PX4_SIM_MODEL=gz_x500_gimbal PX4_GZ_MODEL_POSE=5,0,0,0,0,0 ./build/px4_sitl_default/bin/px4 -i 1

Need to change the instance and pose according to the drone spawned. I want the drones to spawn behind one another having a distance of 5. I also want to setup the mavros nodes for each of the drone. I am using the follwoing command to setup 1 mavros node in terminal:
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14580 -r __ns:=/drone1

Need to change the namespace and the port according to the respective drones.'''