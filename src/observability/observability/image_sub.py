import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from functools import partial


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.li_observability_subscription = self.create_subscription(
            Image,
            'li_observability_matrix',
            partial(self.image_callback, name='lidar_observability', encoding='mono8'),
            10)
        self.li_observability_subscription  # prevent unused variable warning
        self.li_occlusions_subscription = self.create_subscription(
            Image,
            'li_occlusions_matrix',
            partial(self.image_callback, name='lidar_occlusions', encoding='mono8'),
            10)
        self.li_occlusions_subscription  # prevent unused variable warning
        self.li_detailed_subscription = self.create_subscription(
            Image,
            'li_observability_detailed_image',
            partial(self.image_callback, name='lidar_detailed', encoding='bgr8'),
            10)
        self.li_detailed_subscription  # prevent unused variable warning
        self.bb_observability_subscription = self.create_subscription(
            Image,
            'bb_observability_matrix',
            partial(self.image_callback, name='bb_observability', encoding='mono8'),
            10)
        self.bb_observability_subscription  # prevent unused variable warning
        self.bb_occlusions_subscription = self.create_subscription(
            Image,
            'bb_occlusions_matrix',
            partial(self.image_callback, name='bb_occlusions', encoding='mono8'),
            10)
        self.bb_occlusions_subscription  # prevent unused variable warning
        self.bb_detailed_subscription = self.create_subscription(
            Image,
            'bb_observability_detailed_image',
            partial(self.image_callback, name='bb_detailed', encoding='bgr8'),
            10)
        self.bb_detailed_subscription  # prevent unused variable warning


    def image_callback(self, msg, name, encoding):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        cv.imshow(name, image)
        cv.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()