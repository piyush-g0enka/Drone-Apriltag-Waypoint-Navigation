import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.subscription = self.create_subscription(
            PoseStamped,
            'tag_detections/tagpose_inertial',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Path, 'path_waypoints', 10)
        self.poses = {0: None, 1: None, 2: None}
        self.timer = self.create_timer(1.0, self.publish_path)

    def listener_callback(self, msg):
        if not msg.header.frame_id == '':
            frame_id = int(msg.header.frame_id)
            if frame_id in self.poses:
                self.poses[frame_id] = msg
                self.get_logger().info(f'Received pose for object {frame_id}')

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'world'
        path_msg.poses = [pose for pose in self.poses.values() if pose is not None]
        self.publisher.publish(path_msg)
        self.get_logger().info('Published path message')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
