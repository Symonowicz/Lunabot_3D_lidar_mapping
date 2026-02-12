import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # Frames we are bridging
        self.source_parent = 'camera_init'
        self.source_child = 'aft_mapped'
        self.target_parent = 'odom'
        self.target_child = 'base_link'

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.br = TransformBroadcaster(self)

        # Publish at 50 Hz
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            # Look up latest camera_init -> aft_mapped transform
            t = self.buffer.lookup_transform(
                self.source_parent,
                self.source_child,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )

            # Re-broadcast as odom -> base_link
            out = TransformStamped()
            out.header.stamp = t.header.stamp
            out.header.frame_id = self.target_parent
            out.child_frame_id = self.target_child
            out.transform = t.transform

            self.br.sendTransform(out)

        except Exception as e:
            # During startup it will occasionally not find the transform; ignore
            self.get_logger().debug(f'Failed to lookup transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
