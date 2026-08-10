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

    def labeled_path_callback(self, msg: LabeledPathArray):
        marker_array = MarkerArray()

        if not msg.paths:
            # Empty message = "clear the path" (planning failed or path discarded).
            # An empty MarkerArray would delete nothing, so send an explicit DELETEALL.
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            self.get_logger().info('Received empty LabeledPathArray; cleared path markers.')
            return

        frame = msg.paths[0].header.frame_id

        # --- 1. Delete all previous markers ---
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = frame
        marker_array.markers.append(delete_marker)
        
        print("Received labels:", list(msg.labels))

        for seg_idx, (path, label) in enumerate(zip(msg.paths, msg.labels)):
            if not path.poses:
                continue

            rgba = LABEL_COLORS.get(label, (1.0, 1.0, 1.0, 1.0))
            for i, pose_stamped in enumerate(path.poses):
                arrow_marker = Marker()
                arrow_marker.header = path.header
                # One FIXED namespace with unique ids. A namespace per arrow
                # (arrow_<global counter>) made rviz persist thousands of
                # namespace toggles into every saved config file.
                arrow_marker.ns = 'path_arrows'
                arrow_marker.id = seg_idx * 100000 + i
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
