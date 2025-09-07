import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs


class RedCircleDetector(Node):
    def __init__(self):
        super().__init__('red_circle_detector')
        self.bridge = CvBridge()
        self.marker_id = 0
        self.fx = self.fy = self.cx = self.cy = None

        # Topics
        self.rgb_sub = self.create_subscription(Image, '/drone1/rgb_camera/image_raw', self.rgb_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/drone1/depth_camera/image_raw', self.depth_cb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/drone1/rgb_camera/camera_info', self.cam_info_cb, 10)

        # Marker publisher
        self.marker_pub = self.create_publisher(Marker, '/drone1/red_circle_markers', 10)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest images
        self.latest_rgb = None
        self.latest_depth = None

    def cam_info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def rgb_cb(self, msg):
        self.latest_rgb = msg
        self.try_detect()

    def depth_cb(self, msg):
        self.latest_depth = msg
        self.try_detect()

    def try_detect(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return
        if self.fx is None:
            return

        rgb_img = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')  # 16UC1 or 32FC1

        # === RED DETECTION ===
        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
        red_mask = mask1 | mask2

        # === HOUGH CIRCLES ===
        red_blur = cv2.GaussianBlur(red_mask, (9, 9), 2)
        circles = cv2.HoughCircles(red_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40,
                                   param1=50, param2=15, minRadius=10, maxRadius=200)

        if circles is not None:
            for (x, y, r) in np.round(circles[0, :]).astype("int"):
                # === Get depth at center
                if y >= depth_img.shape[0] or x >= depth_img.shape[1]:
                    continue
                depth = float(depth_img[y, x])
                if depth == 0.0 or np.isnan(depth):
                    continue

                # === Backproject to 3D
                X = (x - self.cx) * depth / self.fx
                Y = (y - self.cy) * depth / self.fy
                Z = depth

                cam_point = PointStamped()
                cam_point.header.frame_id = 'drone1/camera_link'
                cam_point.header.stamp = self.latest_depth.header.stamp
                cam_point.point.x = X
                cam_point.point.y = Y
                cam_point.point.z = Z

                try:
                    tf = self.tf_buffer.lookup_transform('map', cam_point.header.frame_id, rclpy.time.Time())
                    map_point = tf2_geometry_msgs.do_transform_point(cam_point, tf)
                    self.publish_marker(map_point)
                except Exception as e:
                    self.get_logger().warn(f'TF lookup failed: {e}')

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_circles"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        self.marker_pub.publish(marker)
        self.marker_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = RedCircleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
