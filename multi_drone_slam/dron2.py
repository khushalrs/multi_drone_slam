#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
import numpy as np
import math
from .trajectory_generator import get_trajectory_with_pose, resample_path_equal_arc_length
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('offboard_takeoff')
        # parameters
        self.declare_parameter('takeoff_height', 2.0)
        self.takeoff_z = self.get_parameter('takeoff_height').value
        
        # trajectory parameters
        self.declare_parameter('center_x', 50.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('flight_radius', 30.0)
        self.declare_parameter('total_height', 50.0)
        self.declare_parameter('num_sweeps', 4)
        self.declare_parameter('points_per_sweep', 50)
        self.declare_parameter('waypoint_threshold', 0.3)  # Distance threshold to consider waypoint reached
        self.declare_parameter('transition_step_size', 0.1)  # Maximum step size during transition
        self.declare_parameter('desired_speed', 1.0)
        self.declare_parameter('resample_spacing', 0.20)
        self.declare_parameter('recovery_distance_threshold', 2.0)
        
        # state
        self.current_state = State()
        self.connected = False
        self.offboard_set = False
        self.armed = False
        self.current_position = [0.0, 0.0, 0.0]
        self.spawn_position = None  # Initialize to None, will capture actual spawn position
        
        # trajectory
        self.path_xyzrpy = None
        self.milestones_xyzrpy = None
        self.segment_lengths = None
        self.path_yaw_unwrapped = None
        self.seg_idx = 0
        self.seg_s = 0.0
        self.path_complete = False
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
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        self.tick = 0
        self.log_counter = 0  # Counter to limit logging frequency
        self.current_target_pose = None

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
            flip_y=False,
            resample_spacing=None
        )
        
        # Skip the first 20 points of the trajectory
        trimmed = trajectory[20:]
        self.get_logger().info(f'Removing first 20 points from trajectory. Original size: {len(trajectory)}, new size: {len(trimmed)}')
        self.prepare_resampled_path(trimmed)
        return trimmed

    def prepare_resampled_path(self, milestones):
        """Store original milestones and build constant-spacing path for the follower."""
        if milestones is None or len(milestones) == 0:
            self.milestones_xyzrpy = None
            self.path_xyzrpy = None
            self.segment_lengths = None
            self.path_yaw_unwrapped = None
            self.seg_idx = 0
            self.seg_s = 0.0
            self.path_complete = True
            self.current_target_pose = None
            return

        self.milestones_xyzrpy = milestones
        spacing = max(self.get_parameter('resample_spacing').value, 1e-3)
        self.path_xyzrpy = resample_path_equal_arc_length(milestones, spacing)

        if self.path_xyzrpy is None or len(self.path_xyzrpy) == 0:
            self.segment_lengths = None
            self.path_yaw_unwrapped = None
            self.seg_idx = 0
            self.seg_s = 0.0
            self.path_complete = True
            self.current_target_pose = None
            return

        if len(self.path_xyzrpy) >= 2:
            self.segment_lengths = np.linalg.norm(
                np.diff(self.path_xyzrpy[:, :3], axis=0), axis=1
            )
        else:
            self.segment_lengths = np.array([])

        self.path_yaw_unwrapped = np.unwrap(self.path_xyzrpy[:, 5])
        self.seg_idx = 0
        self.seg_s = 0.0
        self.path_complete = False
        self.current_target_pose = None

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def advance_cursor(self, distance):
        if self.path_xyzrpy is None or self.segment_lengths is None:
            return

        if len(self.segment_lengths) == 0:
            self.path_complete = True
            return

        remaining = max(distance, 0.0)
        epsilon = 1e-6
        num_segments = len(self.segment_lengths)

        while remaining > epsilon and self.seg_idx < num_segments:
            seg_len = self.segment_lengths[self.seg_idx]
            if seg_len <= epsilon:
                self.seg_idx += 1
                self.seg_s = 0.0
                continue

            available = seg_len - self.seg_s
            if remaining < available - epsilon:
                self.seg_s += remaining
                remaining = 0.0
            else:
                remaining -= available
                self.seg_idx += 1
                self.seg_s = 0.0

        if self.seg_idx >= num_segments:
            self.seg_idx = num_segments - 1
            if self.seg_idx < 0:
                self.seg_idx = 0
            if num_segments > 0:
                self.seg_s = self.segment_lengths[self.seg_idx]
            else:
                self.seg_s = 0.0
            self.path_complete = True
        else:
            self.path_complete = False

    def interpolate_current_segment(self):
        if self.path_xyzrpy is None or len(self.path_xyzrpy) == 0:
            return None

        if self.segment_lengths is None or len(self.segment_lengths) == 0:
            self.path_complete = True
            return self.path_xyzrpy[-1]

        seg_idx = min(max(self.seg_idx, 0), len(self.segment_lengths) - 1)
        seg_len = self.segment_lengths[seg_idx]
        if seg_len <= 1e-6:
            ratio = 1.0
        else:
            ratio = np.clip(self.seg_s / seg_len, 0.0, 1.0)

        p0 = self.path_xyzrpy[seg_idx, :3]
        p1 = self.path_xyzrpy[seg_idx + 1, :3]
        xyz = p0 + (p1 - p0) * ratio

        roll0 = self.path_xyzrpy[seg_idx, 3]
        roll1 = self.path_xyzrpy[seg_idx + 1, 3]
        pitch0 = self.path_xyzrpy[seg_idx, 4]
        pitch1 = self.path_xyzrpy[seg_idx + 1, 4]
        yaw0 = self.path_yaw_unwrapped[seg_idx]
        yaw1 = self.path_yaw_unwrapped[seg_idx + 1]

        roll = roll0 + (roll1 - roll0) * ratio
        pitch = pitch0 + (pitch1 - pitch0) * ratio
        yaw = self.wrap_to_pi(yaw0 + (yaw1 - yaw0) * ratio)

        return np.array([xyz[0], xyz[1], xyz[2], roll, pitch, yaw], dtype=float)

    def compute_target_pose(self):
        if self.path_xyzrpy is None or len(self.path_xyzrpy) == 0:
            return None

        if len(self.path_xyzrpy) == 1:
            self.path_complete = True
            return self.path_xyzrpy[0]

        desired_speed = max(self.get_parameter('desired_speed').value, 0.0)
        advance = desired_speed * self.timer_period
        if advance > 0.0:
            self.advance_cursor(advance)

        return self.interpolate_current_segment()

    def snap_cursor_to_nearest(self, position):
        if self.path_xyzrpy is None or len(self.path_xyzrpy) == 0:
            return False

        diffs = self.path_xyzrpy[:, :3] - position
        distances = np.linalg.norm(diffs, axis=1)
        idx = int(np.argmin(distances))

        if len(self.path_xyzrpy) == 1:
            self.seg_idx = 0
            self.seg_s = 0.0
            self.path_complete = True
            return True

        self.seg_idx = min(idx, len(self.path_xyzrpy) - 2)
        self.seg_s = 0.0
        self.path_complete = False
        self.current_target_pose = None
        return True
        
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
        
    def timer_cb(self):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        
        if self.spawn_position is None and self.connected:
            self.spawn_position = self.current_position.copy()
            self.get_logger().info(f"Stored spawn position: x={self.spawn_position[0]:.2f}, y={self.spawn_position[1]:.2f}, z={self.spawn_position[2]:.2f}")
            self.generate_trajectory()
            if self.path_xyzrpy is not None:
                spacing = self.get_parameter('resample_spacing').value
                self.get_logger().info(
                    f"Resampled path ready with {len(self.path_xyzrpy)} points (spacing≈{spacing:.2f} m)"
                )
        
        if self.flight_state == self.STATE_TAKEOFF and self.spawn_position:
            height_above_spawn = self.current_position[2] - self.spawn_position[2]
            if height_above_spawn >= self.takeoff_z - 0.1:
                self.flight_state = self.STATE_FOLLOW_TRAJECTORY
                self.get_logger().info(f'Takeoff height {self.takeoff_z}m reached! Starting trajectory following...')
        
        if self.flight_state in (self.STATE_INIT, self.STATE_TAKEOFF):
            if self.spawn_position:
                sp.pose.position.x = self.spawn_position[0]
                sp.pose.position.y = self.spawn_position[1]
                sp.pose.position.z = self.spawn_position[2] + self.takeoff_z
            else:
                sp.pose.position.x = 0.0
                sp.pose.position.y = 0.0
                sp.pose.position.z = 0.0
            sp.pose.orientation.w = 1.0
            sp.pose.orientation.x = 0.0
            sp.pose.orientation.y = 0.0
            sp.pose.orientation.z = 0.0
            
            if self.tick % 20 == 0:
                self.get_logger().info(f"SENDING SETPOINT (TAKEOFF): x={sp.pose.position.x:.2f}, y={sp.pose.position.y:.2f}, z={sp.pose.position.z:.2f}")
                    
        elif self.flight_state == self.STATE_FOLLOW_TRAJECTORY:
            target_pose = self.compute_target_pose()
            if target_pose is None:
                hover_z = self.takeoff_z
                if self.spawn_position:
                    sp.pose.position.x = self.spawn_position[0]
                    sp.pose.position.y = self.spawn_position[1]
                    sp.pose.position.z = self.spawn_position[2] + hover_z
                else:
                    sp.pose.position.x = 0.0
                    sp.pose.position.y = 0.0
                    sp.pose.position.z = hover_z
                sp.pose.orientation.w = 1.0
                sp.pose.orientation.x = 0.0
                sp.pose.orientation.y = 0.0
                sp.pose.orientation.z = 0.0
            else:
                current_pos_np = np.array(self.current_position)
                target_pos = target_pose[:3]
                error = np.linalg.norm(target_pos - current_pos_np)
                recovery_threshold = max(self.get_parameter('recovery_distance_threshold').value, 0.0)

                if error > recovery_threshold and self.snap_cursor_to_nearest(current_pos_np):
                    target_pose = self.interpolate_current_segment()
                    target_pos = target_pose[:3]
                    self.get_logger().warn(
                        f'Position error {error:.2f}m exceeds {recovery_threshold:.2f}m, snapping path cursor to index {self.seg_idx}.'
                    )

                self.current_target_pose = target_pose
                sp.pose.position.x = float(target_pos[0])
                sp.pose.position.y = float(target_pos[1])
                sp.pose.position.z = float(target_pos[2])

                yaw = target_pose[5]
                sp.pose.orientation.w = math.cos(yaw / 2.0)
                sp.pose.orientation.x = 0.0
                sp.pose.orientation.y = 0.0
                sp.pose.orientation.z = math.sin(yaw / 2.0)

                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f"SENDING SETPOINT (FOLLOW idx={self.seg_idx}, s={self.seg_s:.2f}): "
                        f"x={target_pos[0]:.2f}, y={target_pos[1]:.2f}, z={target_pos[2]:.2f}, "
                        f"yaw={math.degrees(yaw):.1f}deg"
                    )

                if self.path_complete:
                    self.get_logger().info('Trajectory completed!')
                    self.flight_state = self.STATE_COMPLETED
        else:  # STATE_COMPLETED or unknown
            hold_pose = None
            if self.path_xyzrpy is not None and len(self.path_xyzrpy) > 0:
                hold_pose = self.path_xyzrpy[-1]
            elif self.milestones_xyzrpy is not None and len(self.milestones_xyzrpy) > 0:
                hold_pose = self.milestones_xyzrpy[-1]

            if hold_pose is not None:
                sp.pose.position.x = float(hold_pose[0])
                sp.pose.position.y = float(hold_pose[1])
                sp.pose.position.z = float(hold_pose[2])
                yaw = hold_pose[5]
                sp.pose.orientation.w = math.cos(yaw / 2.0)
                sp.pose.orientation.x = 0.0
                sp.pose.orientation.y = 0.0
                sp.pose.orientation.z = math.sin(yaw / 2.0)
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f"SENDING SETPOINT (HOLD): x={hold_pose[0]:.2f}, "
                        f"y={hold_pose[1]:.2f}, z={hold_pose[2]:.2f}"
                    )
            else:
                if self.spawn_position:
                    sp.pose.position.x = self.spawn_position[0]
                    sp.pose.position.y = self.spawn_position[1]
                    sp.pose.position.z = self.spawn_position[2] + self.takeoff_z
                else:
                    sp.pose.position.x = 0.0
                    sp.pose.position.y = 0.0
                    sp.pose.position.z = self.takeoff_z
                sp.pose.orientation.w = 1.0
                sp.pose.orientation.x = 0.0
                sp.pose.orientation.y = 0.0
                sp.pose.orientation.z = 0.0
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f"SENDING SETPOINT (HOVER-SPAWN): x={sp.pose.position.x:.2f}, "
                        f"y={sp.pose.position.y:.2f}, z={sp.pose.position.z:.2f}"
                    )
                
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
