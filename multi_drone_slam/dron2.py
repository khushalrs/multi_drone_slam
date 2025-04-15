import rclpy
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
        self.offboard_control_publisher = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', 10)
        
        # Subscriber
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/px4_2/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleControlMode()
        self.takeoff_altitude = 20.0  # meters
        self.forward_distance = 20.0  # meters
        self.hover_time = 5.0  # seconds
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
                self.get_logger().info('Reached takeoff altitude, starting hover')

        elif self.state == 'HOVER':
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -self.takeoff_altitude)
            
            if time.time() - self.start_hover_time > self.hover_time:
                self.state = 'FORWARD'
                self.get_logger().info('Hover complete, moving forward')

        elif self.state == 'FORWARD':
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(-10.0, self.forward_distance, -self.takeoff_altitude)
            
            if (abs(self.forward_distance) < 1.0):  # Check if drone is close to target
                self.state = 'LAND'
                self.get_logger().info('Forward movement complete, initiating landing')

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