#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class GeoVisualizer(Node):
    def __init__(self):
        super().__init__('geo_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, '/geo_transformer/markers', 10)
        self.create_subscription(Point, '/geo_transformer/visualize_point', self.point_callback, 10)
        self.create_subscription(Point, '/geo_transformer/origin', self.origin_callback, 10)
        self.points = []
        self.origin = None

    def point_callback(self, msg):
        print(f"[Visualizer] Received point: x={msg.x}, y={msg.y}, z={msg.z}")
        self.points.append(msg)
        self.publish_markers()

    def origin_callback(self, msg):
        print(f"[Visualizer] Received origin: x={msg.x}, y={msg.y}, z={msg.z}")
        self.origin = msg
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        # Points
        for i, pt in enumerate(self.points):
            # Green sphere for the point
            m = Marker()
            m.header.frame_id = 'odom'  # Use 'odom' for default RViz config
            m.header.stamp = self.get_clock().now().to_msg()
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.id = i
            m.pose.position = pt
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0  # Larger for visibility
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            marker_array.markers.append(m)
            # Add a text label above the point
            t = Marker()
            t.header.frame_id = 'odom'
            t.header.stamp = self.get_clock().now().to_msg()
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.id = 1000 + i
            t.pose.position.x = pt.x
            t.pose.position.y = pt.y
            t.pose.position.z = pt.z + 1.0  # Above the sphere
            t.pose.orientation.w = 1.0
            t.scale.z = 0.5  # Text height
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 1.0
            t.text = f"P{i}"
            t.lifetime.sec = 0
            t.lifetime.nanosec = 0
            marker_array.markers.append(t)
        # Origin
        if self.origin:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.id = 10000
            m.pose.position = self.origin
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3  # Smaller origin
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            marker_array.markers.append(m)
            # Add a text label for the origin
            t = Marker()
            t.header.frame_id = 'odom'
            t.header.stamp = self.get_clock().now().to_msg()
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.id = 11000
            t.pose.position.x = self.origin.x
            t.pose.position.y = self.origin.y
            t.pose.position.z = self.origin.z + 0.5
            t.pose.orientation.w = 1.0
            t.scale.z = 0.5
            t.color.r = 1.0
            t.color.g = 0.0
            t.color.b = 0.0
            t.color.a = 1.0
            t.text = "Origin"
            t.lifetime.sec = 0
            t.lifetime.nanosec = 0
            marker_array.markers.append(t)
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = GeoVisualizer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
