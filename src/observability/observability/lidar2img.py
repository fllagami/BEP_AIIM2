import numpy as np
import cv2 as cv
from scipy import ndimage
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import json
import os
from cv_bridge import CvBridge

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

WHITE = np.array((255, 255, 255), np.uint8)
RED = np.array((0, 0, 255), np.uint8)
BLACK = np.array((0, 0, 0), np.uint8)
GREEN = np.array((0, 255, 0), np.uint8)
BLUE = np.array((255, 0, 0), np.uint8)


def make_obs_lidar(pcl2, viewer_pos=(0, 0), viewer_height=1.7, sensing_range=80, scale=0.5,
                 visibility_ang=np.pi * 2 / 3, size_final=(84, 84), shift=True):
    """
        @param pcl2: Pointcloud2 to read from.
        @param viewer_pos: Position of viewer in meters.
        @param viewer_height: Height of viewer in meters.
        @param sensing_range: In meters.
        @param scale: Meters per pixel.
        @param visibility_ang: Angle range of visibility in rad. Set to None if not needed.
        @param size_final: Size of final image to be used as AI input in pixels.
        @param shift: Bool to shift image in order to have viewer on center bottom or not.
    """
    # make scale 0.5 and 0.2
    # also publish only bounding boxes
    bridge = CvBridge()

    if scale < 1:
        scale_multiplier = int(1 / scale)
    else:
        scale_multiplier = 1

    axis = int((size_final[0] // sensing_range + 2) * sensing_range * scale * scale_multiplier)

    occlusions = np.zeros((axis, axis * 2), dtype=np.uint8)  # (height, width)

    for p in read_points(pcl2, ['x', 'y', 'z'], skip_nans=True):
        if abs(int(p[0])) < ((sensing_range + viewer_pos[0]) // scale_multiplier) \
                and abs(int(p[1])) < ((sensing_range + viewer_pos[1]) // scale_multiplier) \
                and p[2] >= viewer_height:
            occlusions[(int(p[0] * scale_multiplier) - axis) * -1,
                       (int(p[1] * scale_multiplier) - axis) * -1] = p[2]

    if viewer_pos is None:
        viewer_pos = (axis, axis)
    else:
        viewer_pos = (int(axis - viewer_pos[0] * scale_multiplier), int(axis - viewer_pos[1] * scale_multiplier))

    # make observability mask
    observability = view_field(occlusions, viewer_pos, sensing_range * scale_multiplier)

    # mark unobservable places due to visibility angle and viewer position
    position_mask = np.full((axis, axis * 2), False)
    if not shift:
        position_mask[viewer_pos[0]: axis, 0: axis * 2] = True
    if visibility_ang is not None:
        angle = (np.pi - visibility_ang) / 2
        for w in range(1, viewer_pos[1], 1):
            position_mask[viewer_pos[0] - int(round(np.tan(angle) * w)): viewer_pos[0], viewer_pos[1] - w] = True
        for w in range(1, axis * 2 - viewer_pos[1], 1):
            position_mask[viewer_pos[0] - int(round(np.tan(angle) * w)): viewer_pos[0], viewer_pos[1] + w] = True

    # color occlusions and observability map
    inclusive_image = np.where(observability[..., None], WHITE, BLACK)
    inclusive_image[viewer_pos[0] - 2: viewer_pos[0], viewer_pos[1] - 1: viewer_pos[1] + 1] = GREEN
    inclusive_image = np.where((occlusions == 0)[..., None], inclusive_image, RED)
    inclusive_image = np.where(position_mask[..., None], BLACK, inclusive_image)

    # color observability map
    observability = np.where(observability, np.uint8(255), np.uint8(0))
    observability = np.where(position_mask, np.uint8(0), observability)

    # color occlusions map
    occlusions = np.where(position_mask, np.uint8(0), occlusions)

    # scale and show images
    if shift:
        midpoint = viewer_pos
    else:
        midpoint = (axis, axis)

    if scale < 1:
        observability = observability[int(midpoint[0] - size_final[0]): int(midpoint[0]),
                    int(midpoint[1] - size_final[1] / 2):
                    int(midpoint[1] + size_final[1] / 2)]

        occlusions = occlusions[int(midpoint[0] - size_final[0]): int(midpoint[0]),
                     int(midpoint[1] - size_final[1] / 2):
                     int(midpoint[1] + size_final[1] / 2)]

        inclusive_image = inclusive_image[int(midpoint[0] - size_final[0]): int(midpoint[0]),
                                int(midpoint[1] - size_final[1] / 2):
                                int(midpoint[1] + size_final[1] / 2)]
        inclusive_image = cv.resize(inclusive_image, (500, 500), interpolation=cv.INTER_LINEAR)
    else:
        observability = observability[int(midpoint[0] - size_final[0] * scale): int(midpoint[0]),
                    int(midpoint[1] - size_final[1] * scale / 2):
                    int(midpoint[1] + size_final[1] * scale / 2)]
        observability = observability[::scale, ::scale]

        occlusions = occlusions[int(midpoint[0] - size_final[0] * scale): int(midpoint[0]),
                     int(midpoint[1] - size_final[1] * scale / 2):
                     int(midpoint[1] + size_final[1] * scale / 2)]
        occlusions = occlusions[::scale, ::scale]

        inclusive_image = inclusive_image[int(midpoint[0] - size_final[0] * scale): int(midpoint[0]),
                                int(midpoint[1] - size_final[1] * scale / 2):
                                int(midpoint[1] + size_final[1] * scale / 2)]
        inclusive_image = cv.resize(inclusive_image, (500, 500), interpolation=cv.INTER_LINEAR)

    cv.imshow('Lidar observability 84x84', observability)

    cv.imshow('Lidar occlusions 84x84', occlusions)

    cv.imshow('Lidar observability and occlusions', inclusive_image)

    cv.waitKey(10)

    return bridge.cv2_to_imgmsg(observability, encoding='mono8'), \
           bridge.cv2_to_imgmsg(occlusions, encoding='mono8'), \
           bridge.cv2_to_imgmsg(inclusive_image, encoding='bgr8')


def save_img(img, name, counter=None):
    path = os.path.join(os.path.join(os.path.expanduser('~')), 'Desktop/Images')
    if counter is not None:
        cv.imwrite(os.path.join(path, '{}{}.jpg'.format(name, counter)), img)
    else:
        cv.imwrite(os.path.join(path, '{}.jpg'.format(name)), img)


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if
                  field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype)  # , file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    # assert isinstance(cloud,
    # roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, \
                                                       cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def view_field(height_map, viewer_pos, max_distance=100, resolution=50):  # assume 50m range
    height_map = np.asarray(height_map)
    h, w = height_map.shape
    vi, vj = viewer_pos
    # Find locations higher than viewer
    m = (height_map != 0)  # & (height_map < 7)
    # Find angle of each pixel relative to viewer
    ii, jj = np.ogrid[-vi:h - vi, -vj:w - vj]
    a = np.arctan2(ii, jj)
    # Distance of each pixel to viewer
    d2 = np.square(ii) + np.square(jj)
    d = np.sqrt(d2)

    # Find angle range "behind" each pixel
    pix_size = 0
    ad = np.arccos(d / np.sqrt(d2 + np.square(pix_size) + 0.001))
    # Minimum and maximum angle encompassed by each pixel
    amin = a - ad
    amax = a + ad
    # Define angle "bins"
    ar = np.linspace(-np.pi, np.pi, resolution + 1)
    # Find the bin corresponding to each pixel
    b = np.digitize(a, ar) % resolution
    bmin = np.digitize(amin, ar) % resolution
    bmax = np.digitize(amax, ar) % resolution
    # Find the closest distance to a high pixel for each angle bin
    angdist = np.full_like(ar, np.inf)
    np.minimum.at(angdist, bmin[m], d[m])
    np.minimum.at(angdist, bmax[m], d[m])
    # Visibility is true if the pixel distance is less than the
    # visibility distance for its angle bin
    return (d <= angdist[b]) & (d <= max_distance)
