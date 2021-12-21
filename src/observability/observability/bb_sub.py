import rclpy
from rclpy.node import Node
from aiim_autoware_msgs.msg import BoundingBoxArray
from aiim_autoware_msgs.msg import VehicleKinematicState
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from observability.bb2img import make_obs_bb
from datetime import datetime


class BBoxSubscriber(Node):
    img_counter = 0
    viewer_position = (0, 0)
    longitudinal_velocity = 0.0

    def __init__(self):
        super().__init__('bounding_box_observability')
        self.bb_subscription = self.create_subscription(
            BoundingBoxArray,
            'trajectory_bbox_array',
            self.bb_callback,
            10)
        self.bb_subscription  # prevent unused variable warning

        self.state_subscription = self.create_subscription(
            VehicleKinematicState,
            'vehicle_state',
            self.vehicle_state_update,
            1)
        self.state_subscription  # prevent unused variable warning

        self.observability_pub = self.create_publisher(Image, 'bb_observability_matrix', 1)
        self.occlusions_pub = self.create_publisher(Image, 'bb_occlusions_matrix', 1)
        self.inclusive_image_pub = self.create_publisher(Image, 'bb_observability_detailed_image', 10)
        self.ai_result = self.create_publisher(Float64, 'ai_result', 1)

    def bb_callback(self, msg):
        start = datetime.now()

        box_set = []
        for box in msg.boxes:
            box_instance = []
            for i in range(4):
                box_corner = []
                box_corner.append(box.corners[i].y)
                box_corner.append(box.corners[i].x)
                box_instance.append(box_corner)
            box_set.append(box_instance)

        estimations = make_obs_bb(box_set, viewer_pos=self.viewer_position, scale=0.5)

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
    minimal_subscriber = BBoxSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
