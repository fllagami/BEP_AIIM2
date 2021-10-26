import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aiim_autoware_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import PointCloud2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        # self.subscription = self.create_subscription(
        #     BoundingBoxArray,
        #     'trajectory_bbox_array',
        #     self.bb_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            PointCloud2,
            'rslidar_points',
            self.pc_callback,
            10)
        self.subscription  # prevent unused variable warning

    # def bb_callback(self, msg):
    #     # label = msg[0].vehicle_label
    #     # bbox = BoundingBoxArray()
    #     # bbox = msg
    #     capacity = msg.CAPACITY
    #     self.get_logger().info('vehicle label: "%d"' % capacity)
    #     # self.get_logger().info(bbox)


    def pc_callback(self, msg):
        data = msg.data
        # max = 0
        # for i in data:
        #     if i > max:
                # max = i
        print(data)
        print('\n \n \n \n \n \n \n \n \n \n \n ')
        # convert_2_img(msg)
        # print(data)
        # self.get_logger().info('I heard pc: "%s"' % msg.data)
        # self.get_logger().info('max data: "%d"' % i)

# def convert_2_img(self):


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()