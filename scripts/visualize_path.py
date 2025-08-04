#! /bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from ugv_nav4d_ros2.msg import LabeledPathArray  # Replace with your actual package

LABEL_COLORS = {
    'Forward':   (0.0, 1.0, 0.0, 1.0),  # Green
    'Backward':  (1.0, 0.0, 0.0, 1.0),  # Red
    'PointTurn': (0.0, 0.0, 1.0, 1.0),  # Blue
    'Lateral':   (1.0, 1.0, 0.0, 1.0),  # Yellow
}

class CombinedPathArrowPublisher(Node):
    def __init__(self):
        super().__init__('combined_path_arrow_publisher')
        self.subscription = self.create_subscription(
            LabeledPathArray,
            '/ugv_nav4d_ros2/labeled_path_segments',
            self.labeled_path_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/ugv_nav4d_ros2/colored_path', 10)
        self.get_logger().info('CombinedPathArrowPublisher ready.')

        self.global_marker_count = 0  # For unique namespaces

    def labeled_path_callback(self, msg: LabeledPathArray):
        marker_array = MarkerArray()

        if not msg.paths:
            self.get_logger().warn('No paths in received LabeledPathArray!')
            self.marker_pub.publish(marker_array)
            return

        frame = msg.paths[0].header.frame_id

        # --- 1. Delete all previous markers ---
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = frame
        marker_array.markers.append(delete_marker)

        # --- 2. Add static legend ---
        legend_x = -10.0  # Adjust as needed for your frame!
        legend_y = 10.0
        y_gap = 0.8
        for idx, (label, color) in enumerate(LABEL_COLORS.items()):
            legend_marker = Marker()
            legend_marker.header.frame_id = frame
            legend_marker.header.stamp = self.get_clock().now().to_msg()
            legend_marker.ns = "legend"
            legend_marker.id = idx
            legend_marker.type = Marker.TEXT_VIEW_FACING
            legend_marker.action = Marker.ADD
            legend_marker.pose.position.x = legend_x
            legend_marker.pose.position.y = legend_y - idx * y_gap
            legend_marker.pose.position.z = 0.0
            legend_marker.pose.orientation.w = 1.0
            legend_marker.scale.z = 0.8
            legend_marker.color.r, legend_marker.color.g, legend_marker.color.b, legend_marker.color.a = color
            legend_marker.text = label
            legend_marker.lifetime.sec = 0
            legend_marker.lifetime.nanosec = 0
            marker_array.markers.append(legend_marker)

        print("Received labels:", list(msg.labels))

        for seg_idx, (path, label) in enumerate(zip(msg.paths, msg.labels)):
            if not path.poses:
                continue

            rgba = LABEL_COLORS.get(label, (1.0, 1.0, 1.0, 1.0))
            for i, pose_stamped in enumerate(path.poses):
                self.global_marker_count += 1
                ns = f'arrow_{self.global_marker_count}'
                arrow_marker = Marker()
                arrow_marker.header = path.header
                arrow_marker.ns = ns
                arrow_marker.id = 0
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.scale.x = 0.4   # shaft length
                arrow_marker.scale.y = 0.1   # shaft diameter
                arrow_marker.scale.z = 0.18  # head diameter
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = rgba
                arrow_marker.pose = pose_stamped.pose  # Use position and orientation from path
                arrow_marker.lifetime.sec = 0
                arrow_marker.lifetime.nanosec = 0
                marker_array.markers.append(arrow_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f'Published {len(marker_array.markers)} markers in MarkerArray. (Legend + Arrows)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CombinedPathArrowPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
