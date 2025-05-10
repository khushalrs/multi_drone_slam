#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np
from .trajectory_generator import get_trajectory_with_pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')
        # parameters
        self.declare_parameter('takeoff_height', 5.0)
        self.takeoff_z = self.get_parameter('takeoff_height').value
        
        # trajectory parameters
        self.declare_parameter('center_x', 50.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('flight_radius', 23.0)
        self.declare_parameter('total_height', 50.0)
        self.declare_parameter('num_sweeps', 6)
        self.declare_parameter('points_per_sweep', 50)
        self.declare_parameter('waypoint_threshold', 0.3)  # Distance threshold to consider waypoint reached
        self.declare_parameter('transition_step_size', 0.1)  # Maximum step size during transition
        
        # state
        self.current_state = State()
        self.connected = False
        self.offboard_set = False
        self.armed = False
        self.current_position = [0.0, 0.0, 0.0]
        self.spawn_position = None  # Initialize to None, will capture actual spawn position
        
        # trajectory
        self.trajectory = None  # Will generate after capturing spawn position
        self.current_waypoint_idx = 0
        self.STATE_INIT = 0
        self.STATE_TAKEOFF = 1
        self.STATE_FOLLOW_TRAJECTORY = 2  # Removed TRANSITION state
        self.STATE_COMPLETED = 3
        self.flight_state = self.STATE_INIT
        # Remove transition_progress variable as it's no longer needed

        # subscribers & publishers
        self.state_sub = self.create_subscription(
            State, '/drone1/state', self.state_cb, 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/drone1/setpoint_position/local', 10)
        
        # Create a QoS profile with BEST_EFFORT reliability for position updates
        position_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped, '/drone1/local_position/pose', self.position_cb, position_qos)

        # service clients
        self.mode_client = self.create_client(SetMode, '/drone1/set_mode')
        self.arm_client  = self.create_client(CommandBool, '/drone1/cmd/arming')

        # wait for services
        self.get_logger().info('Waiting for /drone1/set_mode service...')
        self.mode_client.wait_for_service()
        self.get_logger().info('Waiting for /drone1/cmd/arming service...')
        self.arm_client.wait_for_service()

        # timer to publish setpoints
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.tick = 0
        self.log_counter = 0  # Counter to limit logging frequency

    def generate_trajectory(self):
        """Generate trajectory based on parameters"""
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        flight_radius = self.get_parameter('flight_radius').value
        total_height = self.get_parameter('total_height').value
        num_sweeps = self.get_parameter('num_sweeps').value
        points_per_sweep = self.get_parameter('points_per_sweep').value
        
        self.get_logger().info(f'Generating trajectory: center=({center_x}, {center_y}), radius={flight_radius}, height={total_height}')
        trajectory = get_trajectory_with_pose(
            (center_x, center_y),
            flight_radius, 
            total_height, 
            num_sweeps, 
            points_per_sweep,
            min_height=self.takeoff_z,
            flip_x=True,
            flip_y=False
        )
        
        # Skip the first 20 points of the trajectory
        self.get_logger().info(f'Removing first 20 points from trajectory. Original size: {len(trajectory)}, new size: {len(trajectory[20:])}')
        return trajectory[20:]
        
    def state_cb(self, msg):
        self.current_state = msg
        if not self.connected and msg.connected:
            self.connected = True
            self.get_logger().info('FCU connected!')

    def position_cb(self, msg):
        """Store current position for waypoint tracking"""
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z
        
        # Log position only every 20 callbacks to avoid flooding (1Hz if position updates at 20Hz)
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(f"CURRENT POSITION: x={self.current_position[0]:.2f}, y={self.current_position[1]:.2f}, z={self.current_position[2]:.2f}")
        
    def waypoint_reached(self):
        """Check if the current waypoint has been reached"""
        if self.current_waypoint_idx >= len(self.trajectory):
            return False
            
        waypoint = self.trajectory[self.current_waypoint_idx]
        distance = np.sqrt(
            (waypoint[0] - self.current_position[0])**2 +
            (waypoint[1] - self.current_position[1])**2 +
            (waypoint[2] - self.current_position[2])**2
        )
        # print(f"Distance to waypoint {self.current_waypoint_idx}: {distance:.2f}")
        threshold = self.get_parameter('waypoint_threshold').value
        return distance < threshold
        
    def interpolate_position(self, pos1, pos2, ratio):
        """Interpolate between two positions based on ratio (0-1)"""
        return [
            pos1[0] + (pos2[0] - pos1[0]) * ratio,
            pos1[1] + (pos2[1] - pos1[1]) * ratio,
            pos1[2] + (pos2[2] - pos1[2]) * ratio
        ]

    def timer_cb(self):
        # 1) Always publish the desired setpoint based on current state
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        
        # Capture spawn position first if not already set
        if self.spawn_position is None and self.connected:
            self.spawn_position = self.current_position.copy()
            self.get_logger().info(f"Stored spawn position: x={self.spawn_position[0]:.2f}, y={self.spawn_position[1]:.2f}, z={self.spawn_position[2]:.2f}")
            # Generate trajectory now that we know spawn position
            self.trajectory = self.generate_trajectory()
        
        # Check if takeoff height has been reached
        if self.flight_state == self.STATE_TAKEOFF and self.spawn_position:
            # Check if we've reached the target height (with a small margin)
            height_above_spawn = self.current_position[2] - self.spawn_position[2]
            if height_above_spawn >= self.takeoff_z - 0.1:  # 0.1m margin
                self.flight_state = self.STATE_FOLLOW_TRAJECTORY
                self.get_logger().info(f'Takeoff height {self.takeoff_z}m reached! Starting trajectory following...')
        
        # Set setpoint based on state
        if self.flight_state == self.STATE_INIT or self.flight_state == self.STATE_TAKEOFF:
            # Initial takeoff phase - use spawn position's x,y coordinates
            if self.spawn_position:
                sp.pose.position.x = self.spawn_position[0]
                sp.pose.position.y = self.spawn_position[1]
                sp.pose.position.z = self.spawn_position[2] + self.takeoff_z  # 2.0m above spawn
                sp.pose.orientation.w = 1.0
                
                # Log setpoint (only every 20 ticks)
                if self.tick % 20 == 0:
                    self.get_logger().info(f"SENDING SETPOINT (TAKEOFF): x={self.spawn_position[0]:.2f}, y={self.spawn_position[1]:.2f}, z={sp.pose.position.z:.2f}")
            else:
                # If spawn position not yet known, send default setpoint
                sp.pose.position.x = 0.0
                sp.pose.position.y = 0.0
                sp.pose.position.z = 0.0
                sp.pose.orientation.w = 1.0
                    
        elif self.flight_state == self.STATE_FOLLOW_TRAJECTORY:
            # Follow trajectory
            if self.current_waypoint_idx < len(self.trajectory):
                waypoint = self.trajectory[self.current_waypoint_idx]
                sp.pose.position.x = float(waypoint[0])
                sp.pose.position.y = float(waypoint[1])
                sp.pose.position.z = float(waypoint[2])
                
                # Log setpoint (only every 20 ticks)
                if self.tick % 20 == 0:
                    self.get_logger().info(f"SENDING SETPOINT (TRAJECTORY #{self.current_waypoint_idx}): x={waypoint[0]:.2f}, y={waypoint[1]:.2f}, z={waypoint[2]:.2f}")
                
                # Calculate quaternion from roll, pitch, yaw
                roll, pitch, yaw = waypoint[3], waypoint[4], waypoint[5]
                
                # Simple conversion for demonstration - in reality you'd use a quaternion conversion
                # This is just setting the yaw, assuming roll and pitch are zero
                import math
                sp.pose.orientation.w = math.cos(yaw/2)
                sp.pose.orientation.z = math.sin(yaw/2)
                
                # Check if we've reached the current waypoint
                if self.waypoint_reached():
                    self.current_waypoint_idx += 1
                    self.get_logger().info(f'Waypoint {self.current_waypoint_idx-1} reached! Moving to next waypoint.')
                    
                    if self.current_waypoint_idx >= len(self.trajectory):
                        self.get_logger().info('Trajectory completed!')
                        self.flight_state = self.STATE_COMPLETED
            else:
                self.flight_state = self.STATE_COMPLETED
        else:  # STATE_COMPLETED or unknown
            # Hover in place at last position
            if len(self.trajectory) > 0:
                last_point = self.trajectory[-1]
                sp.pose.position.x = float(last_point[0])
                sp.pose.position.y = float(last_point[1])
                sp.pose.position.z = float(last_point[2])
                
                # Log setpoint (only every 20 ticks)
                if self.tick % 20 == 0:
                    self.get_logger().info(f"SENDING SETPOINT (HOVER): x={last_point[0]:.2f}, y={last_point[1]:.2f}, z={last_point[2]:.2f}")
            else:
                # Fallback
                sp.pose.position.x = 0.0
                sp.pose.position.y = 0.0
                sp.pose.position.z = self.takeoff_z
                
                # Log setpoint (only every 20 ticks)
                if self.tick % 20 == 0:
                    self.get_logger().info(f"SENDING SETPOINT (FALLBACK): x=0.00, y=0.00, z={self.takeoff_z:.2f}")
                    
            sp.pose.orientation.w = 1.0
                
        self.setpoint_pub.publish(sp)

        # 2) If not connected yet, just keep publishing
        if not self.connected:
            return

        # 3) After a few setpoints, switch to OFFBOARD and arm
        self.tick += 1
        if self.tick == 100:  # ~5 seconds at 20 Hz
            # set OFFBOARD mode
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.get_logger().info('Switching to OFFBOARD mode...')
            self.mode_client.call_async(req)

        if self.tick == 110:
            # arm
            req = CommandBool.Request()
            req.value = True
            self.get_logger().info('Arming vehicle...')
            self.arm_client.call_async(req)
            self.flight_state = self.STATE_TAKEOFF

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()