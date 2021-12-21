import rclpy
from rclpy.node import Node
from observability.lidar2img import make_obs_lidar
from aiim_autoware_msgs.msg import VehicleKinematicState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from datetime import datetime


class LidarSubscriber(Node):
    img_counter = 0
    viewer_position = (0, 0)
    longitudinal_velocity = 0.0

    def __init__(self):
        super().__init__('lidar_observability')
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            'rslidar_points',
            self.lidar_callback,
            1)
        self.lidar_subscription  # prevent unused variable warning

        self.state_subscription = self.create_subscription(
            VehicleKinematicState,
            'vehicle_state',
            self.vehicle_state_update,
            1)
        self.state_subscription  # prevent unused variable warning

        self.observability_pub = self.create_publisher(Image, 'li_observability_matrix', 1)
        self.occlusions_pub = self.create_publisher(Image, 'li_occlusions_matrix', 1)
        self.inclusive_image_pub = self.create_publisher(Image, 'li_observability_detailed_image', 10)
        self.ai_result = self.create_publisher(Float64, 'ai_result', 1)

    def lidar_callback(self, pcl2):
        start = datetime.now()

        estimations = make_obs_lidar(pcl2, viewer_pos=self.viewer_position, scale=0.5)

        observability = estimations[0]
        occlusions = estimations[1]
        inclusive_image = estimations[2]
        self.img_counter += 1

        self.observability_pub.publish(observability)
        self.get_logger().info('Publishing: Lidar Observability matrix {}'.format(self.img_counter))

        self.occlusions_pub.publish(occlusions)
        self.get_logger().info('Publishing: Lidar Occlusions matrix {}'.format(self.img_counter))

        self.inclusive_image_pub.publish(inclusive_image)
        self.get_logger().info('Publishing: Lidar Detailed observability image {}'.format(self.img_counter))

        # call ai
        # self.ai_result.publish(ai_result)
        # self.get_logger().info('Publishing: AI result : {}'.format(ai_result))

        time_elapsed = datetime.now() - start
        print('time elapsed = {}.{}'.format(time_elapsed.seconds, time_elapsed.microseconds * 1000))

    def vehicle_state_update(self, msg):
        self.viewer_position = (int(msg.state.x), int(msg.state.y))
        self.longitudinal_velocity = msg.state.longitudinal_velocity_mps
        print('viewer_position = {}, longitudinal_velocity = {}'.format(self.viewer_position, self.longitudinal_velocity))


def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
