import rclpy
from rclpy.node import Node
from pubsub.matrixli2img import make_pic_mat
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64
from datetime import datetime
from sensor_msgs.msg import Image

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

class LidarSubscriber(Node):
    # max_x = 196.720642, min_x = 0.000000, max_y = 114.409874, min_y = -103.016510, max_z = 35.230556, min_z = -22.740181
    x_max, x_min, y_max, y_min, z_max, z_min = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    img_counter = 0
    pcl_counter = 0
    pcl2_storage = []
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'rslidar_points',
            self.pc_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.obs_pub = self.create_publisher(Image, 'observability_matrix', 1)
        self.rviz2_image_pub = self.create_publisher(Image, 'rviz_obs_image', 10)
        self.ai_result = self.create_publisher(Float64, 'ai_result', 1)


    def pc_callback(self, msg):
        # PointField
        # string name  # Name of field
        # uint32 offset  # Offset from start of point struct
        # uint8 datatype  # Datatype enumeration, see above
        # uint32 count  # How many elements in the field
        # print('fields num = {}, name 1 = {}, count 1 = {}, name 2 = {}, count 2 = {}, name 3 = {}, count 3 = {},'
        #       ' name 4 = {}, count 4 = {}, datatype = {}'.
        #       format(len(msg.fields),
        #              msg.fields[0].name, msg.fields[0].count,
        #              msg.fields[1].name, msg.fields[1].count,
        #              msg.fields[2].name, msg.fields[2].count,
        #              msg.fields[3].name, msg.fields[3].count,
        #              msg.fields[0].datatype))
        # print('data len = {}, first element = {}, row step = {}, height = {}, width = {}, is dense = {}'
        #       .format(len(msg.data), msg.data[0], msg.row_step, msg.height, msg.width, msg.is_dense))
        # rslidar builtin_interfaces.msg.Time(sec=1632835941, nanosec=294447) frami 0
        # rslidar builtin_interfaces.msg.Time(sec=1632836016, nanosec=501603603) frami 755
        # 75 sec
        # # for saving img
        # if self.pcl_counter == 0:
        #     print(msg.header.frame_id, msg.header.stamp, 'e paraaaaaaaaaaaaaaaaaaaaaaaaaa')
        # if self.pcl_counter == 755 or self.pcl_counter == 754 or self.pcl_counter == 756:
        #     print(msg.header.frame_id, msg.header.stamp, 'e fundittttttttttttt frami {}'.format(self.pcl_counter))
        # self.pcl2_storage.append(msg)
        # self.pcl_counter += 1
        # print(self.pcl_counter)
        # if len(self.pcl2_storage) == 756:
        #     self.process()
        start = datetime.now()
        observability = make_pic_mat(msg, scale=0.5)
        # observability = make_pic_mat(msg, scale=0.2)
        time_elapsed = datetime.now() - start
        print('time elapsed = {}.{}'.format(time_elapsed.seconds, time_elapsed.microseconds * 1000))

        obs_matrix = observability[0]
        rviz2_image = observability[1]
        self.img_counter += 1
        # self.obs_pub.publish(obs_matrix)
        # self.get_logger().info('Publishing: Observability matrix')

        # self.rviz2_image_pub.publish(rviz2_image)
        # self.get_logger().info('Publishing: Rviz2 Image {}'.format(self.img_counter))
        ## call ai
        ## self.ai_result.publish(ai_result)
        ## self.get_logger().info('Publishing: AI result : {}'.format(ai_result))

        # lidar_points = []
        # for p in self.read_points(msg, ['x', 'y', 'z'], skip_nans=True):
            # print('x = {}, y = {}, z = {}'.format(p[0], p[1], p[2]))
            # self.is_min_max(p[0], p[1], p[2])
            # point = []
            # point.append(p[0])
            # point.append(p[1])
            # point.append(p[2])
        #     lidar_points.append(p)
        # make_pic(lidar_points)
        # Serialize data into file:
        # json.dump(lidar_points, open("lpoints.json", 'w'))

        # Read data from file:
        # lpoints = json.load(open("lpoints.json"))
        # make_pic(lpoints)
        ######

    def process(self):
        self.pcl_counter += 1
        for pcl2 in self.pcl2_storage:
            print('{} / {}'.format(self.img_counter+1, self.pcl_counter))
            make_pic_mat(pcl2, self.img_counter)
            self.img_counter += 1

    def is_min_max(self, x, y, z):
        if x > self.x_max : self.x_max = x
        if x < self.x_min : self.x_min = x
        if y > self.y_max : self.y_max = y
        if y < self.y_min : self.y_min = y
        if z > self.z_max : self.z_max = z
        if z < self.z_min : self.z_min = z
        print('max_x = %f, min_x = %f, max_y = %f, min_y = %f, max_z = %f, min_z = %f'
              % (self.x_max, self.x_min, self.y_max, self.y_min, self.z_max, self.z_min))

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    rclpy.spin_once(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()