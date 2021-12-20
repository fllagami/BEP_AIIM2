import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            # 'observability_matrix',
            'rviz_obs_image',
            self.matrix_callback,
            10)
        self.image_subscription  # prevent unused variable warning
        # self.rviz2_img_subscription = self.create_subscription(
        #     Image,
        #     'rviz_obs_image',
        #     self.rviz2_image_callback,
        #     10)
        # self.rviz2_img_subscription  # prevent unused variable warning


    def matrix_callback(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv.imshow('lidar', image)
        cv.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()