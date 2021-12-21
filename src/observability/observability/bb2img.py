import numpy as np
import cv2 as cv
from scipy import ndimage
import json
from cv_bridge import CvBridge

# BoxSet = list[list[Point32]]
# Box = list[Point32]

# max_x = 93.292244, min_x = 0.000000
# max_y = 43.019192, min_y = -64.461418

#node output nxn matrix
#80x80 adjustable
#convert distance to px mapping; 1 m to 1 px or 4m to 1 px
#req: 84x84, range 90m
# make viewer position adjustable

WHITE = np.array((255, 255, 255), np.uint8)
RED = np.array((0, 0, 255), np.uint8)
BLACK = np.array((0, 0, 0), np.uint8)
GREEN = np.array((0, 255, 0), np.uint8)
BLUE = np.array((255, 0, 0), np.uint8)

def make_obs_bb(box_set, viewer_pos=(0,0), viewer_height=1.7, sensing_range=80, scale=2,
                   visibility_ang=np.pi*2/3, size_final=(84,84), shift=True):
    """
        @param box_set: Set of bounding boxes to read from.
        @param viewer_pos: Position of viewer in meters.
        @param viewer_height: Height of viewer in meters.
        @param sensing_range: In meters.
        @param scale: Meters per pixel.
        @param visibility_ang: Angle range of visibility in rad.
        @param size_final: Size of final image to be used as AI input in pixels.
        @param shift: Bool to shift image in order to have viewer on center bottom or not.
    """
    bridge = CvBridge()
    if scale < 1:
        scale_multiplier = int(1 / scale)
    else:
        scale_multiplier = 1

    axis = int((size_final[0] // sensing_range + 2) * sensing_range * scale * scale_multiplier)

    occlusions = np.zeros((axis, axis*2), dtype=np.uint8)  # (height, width), only observability
    # inclusive_image = np.zeros((axis, axis*2, 3), dtype=np.uint8)  # (height, width), only observability

    for box in box_set:
        box_polys = map_corners(box, axis, scale_multiplier)
        cv.fillPoly(occlusions, [box_polys], 3)
        # cv.fillPoly(inclusive_image, [box_polys], RED)

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
            position_mask[viewer_pos[0] - round(np.tan(angle) * w): viewer_pos[0], viewer_pos[1] - w] = True
        for w in range(1, axis * 2 - viewer_pos[1], 1):
            position_mask[viewer_pos[0] - round(np.tan(angle) * w): viewer_pos[0], viewer_pos[1] + w] = True

    # color occlusions and observability map
    inclusive_image = np.where(observability[..., None], WHITE, BLACK)
    inclusive_image = np.where(position_mask[..., None], BLACK, inclusive_image)
    inclusive_image[viewer_pos[0] - 2: viewer_pos[0], viewer_pos[1] - 1: viewer_pos[1] + 1] = GREEN
    inclusive_image = np.where((occlusions == 0)[..., None], inclusive_image, RED)

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

    # cv.imshow('BB observability 84x84', observability)
    #
    # cv.imshow('BB occlusions 84x84', occlusions)
    #
    # cv.imshow('BB observability and occlusions', all_included_image)

    cv.waitKey(10)

    return bridge.cv2_to_imgmsg(observability, encoding='mono8'), \
           bridge.cv2_to_imgmsg(occlusions, encoding='mono8'), \
           bridge.cv2_to_imgmsg(inclusive_image, encoding='bgr8')


def map_corners(box, axis, scale_multiplier):
    new_box = []
    for i in range(4):
        new_corners = []
        new_corners.append(int((box[i][0] * scale_multiplier - axis) * -1))
        new_corners.append(int((box[i][1] * scale_multiplier - axis) * -1))
        new_box.append(np.array(new_corners, dtype='int32'))
    return np.array(new_box, dtype='int32')


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
    ad = np.arccos(d / np.sqrt(d2 + np.square(pix_size)+0.001))
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
