import rclpy
from rclpy.node import Node

from aiim_autoware_msgs.msg import BoundingBox
from aiim_autoware_msgs.msg import BoundingBoxArray


class MinimalSubscriber(Node):

    #max_x = 93.292244, min_x = 0.000000, max_y = 43.019192, min_y = -64.461418

    x_max, x_min, y_max, y_min = 0.0, 0.0, 0.0, 0.0

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            'trajectory_bbox_array',
            self.bb_callback,
            10)
        self.subscription  # prevent unused variable warning

        # self.subscription = self.create_subscription(
        #     PointCloud2,
        #     'rslidar_points',
        #     self.pc_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

    def bb_callback(self, msg):
        bbox = BoundingBox()
        for box in msg.boxes:
            # bbox = msg.boxes[1]
            # corners = Point32[4]
            # corners = bbox.corners
            # capacity = msg.CAPACITY
            i = 0
            for corner in box.corners:
                x = box.corners[i].x
                y = box.corners[i].y
                z = box.corners[i].z
                self.is_min_max(x, y)
                i += 1
                print('corner %d: x = %f, y = %f, z = %f' % (i, x, y, z))
            cx = box.centroid.x
            cy = box.centroid.y
            cz = box.centroid.z
            sx = box.size.x
            sy = box.size.y
            sz = box.size.z
            print('centroid: x = %f, y = %f, z = %f' % (cx, cy, cz))
            print('size: x = %f, y = %f, z = %f' % (sx, sy, sz))
            print('max_x = %f, min_x = %f, max_y = %f, min_y = %f' % (self.x_max, self.x_min, self.y_max, self.y_min))

            print('endddddddddddddd')
            # rclpy.shutdown()
        print('ooooooooooooooooooooooooooooooooooooooooooooooo')


    def pc_callback(self, msg):
        data = msg.data
        max = 0
        for i in data:
            if i > max:
                max = i
        print(data)
        print('\n \n \n \n \n \n \n \n \n \n \n ')
        print(data)
        self.get_logger().info('I heard pc: "%s"' % msg.data)
        self.get_logger().info('max data: "%d"' % i)

    def is_min_max(self, x, y):
        if x > self.x_max: self.x_max = x
        if x < self.x_min: self.x_min = x
        if y > self.y_max: self.y_max = y
        if y < self.y_min: self.y_min = y


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()