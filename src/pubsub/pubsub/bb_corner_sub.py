import rclpy
from rclpy.node import Node
import json

from geometry_msgs.msg import Point32
from aiim_autoware_msgs.msg import BoundingBox
from aiim_autoware_msgs.msg import BoundingBoxArray
from pubsub.bb2img import make_pic_bb

# class CornerPublisher(Node):
#     def __init__(self):
#         super().__init__('corner publisher')
#         self.publisher_ = self.create_publisher()

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            'trajectory_bbox_array',
            self.bb_callback,
            10)
        self.subscription  # prevent unused variable warning

    def bb_callback(self, msg):
        box_set = []
        for box in msg.boxes:
            # box_set.append(box.corners)
            box_instance = []
            for i in range(4):
                box_corner = []
                box_corner.append(box.corners[i].y)
                box_corner.append(box.corners[i].x)
                box_instance.append(box_corner)
            box_set.append(box_instance)
        # Serialize data into file:
        # json.dump(box_set, open("boxset.json", 'w'))

        # Read data from file:
        # boxes = json.load(open("boxset.json"))
        make_pic_bb(box_set, scale=0.5)
        make_pic_bb(box_set, scale=0.2)
        # make_pic(boxes)

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