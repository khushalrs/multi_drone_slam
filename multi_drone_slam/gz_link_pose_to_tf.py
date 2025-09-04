#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class GzLinkPoseToTF(Node):
    def __init__(self, child_frame_id: str, pose_topic: str, map_frame: str = 'map'):
        super().__init__(f'{child_frame_id.replace("/", "_")}_tf_broadcaster')
        self.child = child_frame_id
        self.map_frame = map_frame
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, pose_topic, self.cb, 10)
        self.get_logger().info(f'Pose {pose_topic} -> TF {map_frame} -> {child_frame_id}')

    def cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if (msg.header.stamp.sec or msg.header.stamp.nanosec) else self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.child
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    import sys
    if len(sys.argv) < 3:
        print("Usage: gz_link_pose_to_tf <child_frame_id> <pose_topic> [map_frame]")
        return
    child, topic = sys.argv[1], sys.argv[2]
    mapf = sys.argv[3] if len(sys.argv) > 3 else 'map'
    node = GzLinkPoseToTF(child, topic, mapf)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
