# imu_reframe.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ImuReframe(Node):
    def __init__(self):
        super().__init__('imu_reframe')
        in_topic  = self.declare_parameter('in_topic',  '/drone1/imu/data').get_parameter_value().string_value
        out_topic = self.declare_parameter('out_topic', '/drone1/imu/data_fixed').get_parameter_value().string_value
        frame_id  = self.declare_parameter('frame_id',  'drone1/base_link').get_parameter_value().string_value

        # Sensor QoS (BEST_EFFORT) to match typical IMU publishers
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        self.frame_id = frame_id
        self.pub = self.create_publisher(Imu, out_topic, sensor_qos)
        self.sub = self.create_subscription(Imu, in_topic, self.cb, sensor_qos)
        # self.get_logger().info(f"Reframing {in_topic} → {out_topic} with frame_id={frame_id}")

    def cb(self, msg: Imu):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

def main():
    rclpy.init(); n = ImuReframe(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
