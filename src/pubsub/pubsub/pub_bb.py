import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from src.aiim_autoware_msgs.msg import BoundingBoxArray

# Topic: /trajectory_bbox_array |
# Type: aiim_autoware_msgs/msg/BoundingBoxArray | Count: 742 |
# Serialization Format: cdr

# Topic: /rslidar_points |
# Type: sensor_msgs/msg/PointCloud2 | Count: 756
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_bb')
        self.publisher_ = self.create_publisher(BoundingBoxArray, 'bounding_box', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = BoundingBoxArray()
        #msg =
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Bounding Box "%d"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
