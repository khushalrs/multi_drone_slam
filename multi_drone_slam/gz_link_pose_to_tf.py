# multi_drone_slam/odom_to_tf.py
# (exactly your code)
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', 'mavros/local_position/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child  = self.get_parameter('child_frame').get_parameter_value().string_value

        # Create a QoS profile compatible with MAVROS odometry publisher (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, qos_profile)
        self.get_logger().info(
            f'odom_to_tf: subscribing {self.odom_topic}; publishing TF {self.parent} -> {self.child}'
        )

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
