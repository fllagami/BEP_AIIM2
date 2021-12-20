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

def make_pic_bb(box_set, viewer_pos=(0,0), viewer_height=1.7, sensing_range=80, scale=2,
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

    axis = (size_final[0]//sensing_range + 2) * sensing_range * scale

    boxes = np.zeros((axis, axis*2), dtype=np.uint8)  # (height, width), only observability
    image_big = np.zeros((axis, axis*2, 3), dtype=np.uint8)  # (height, width), only observability

    for box in box_set:
        # obs = map_corners(box, size)
        box_polys = map_corners(box, axis)
        # cv.fillPoly(img, [box_red], (0, 0, 250))
        cv.fillPoly(boxes, [box_polys], 3)
        cv.fillPoly(image_big, [box_polys], RED)

    if viewer_pos is None:
        viewer_pos = (axis, axis)
    else:
        viewer_pos = (axis - viewer_pos[0], axis - viewer_pos[1])

    # make observability mask
    observability = view_field(boxes, viewer_pos, sensing_range)

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
    image_big = np.where(observability[..., None], WHITE, image_big)
    image_big = np.where(position_mask[..., None], BLACK, image_big)

    # color observability map
    image_sized = np.where(observability, np.uint8(255), np.uint8(0))
    image_sized = np.where(position_mask, np.uint8(0), image_sized)

    # scale and show images
    if shift:
        midpoint = viewer_pos
    else:
        midpoint = (axis, axis)
    image_sized = image_sized[int(midpoint[0] - size_final[0] * scale): int(midpoint[0]),
                int(midpoint[1] - size_final[1] * scale / 2): int(midpoint[1] + size_final[1] * scale / 2)]
    image_sized = image_sized[::scale, ::scale]
    cv.imshow(image_sized, 'image small')
    # cv.imshow('observability 84x84', eagle_obs)
    # save_img(eagle_obs, 'observability 84x84, with no shift')

    # eagle_obs = cv.resize(eagle_obs, (500, 500), interpolation=cv.INTER_LINEAR)
    # cv.imshow('observability big', eagle_obs)

    # eagle_all = eagle_all[int(midpoint[0] - size_final[0]*scale) : int(midpoint[0]),
    #                   int(midpoint[1] - size_final[1]*scale/2) : int(midpoint[1] + size_final[1]*scale/2)]
    # print(np.shape(eagle_all))
    # eagle_all = cv.resize(eagle_all, (500, 500), interpolation=cv.INTER_LINEAR)
    # cv.imshow('observability and occlusions with no shift', eagle_all)
    # save_img(eagle_all, 'observability and occlusions with no shift')
    image_big = image_big[int(midpoint[0] - size_final[0] * scale): int(midpoint[0]),
                  int(midpoint[1] - size_final[1] * scale / 2): int(midpoint[1] + size_final[1] * scale / 2)]
    image_big = cv.resize(image_big, (500, 500), interpolation=cv.INTER_LINEAR)
    cv.imshow(image_big, 'image big')
    # cv.imshow('observability and occlusions with angles and no shift', eagle_all_with_angles)
    # save_img(eagle_all_with_angles, 'observability and occlusions with angles and no shift')

    cv.waitKey(50)

    # return bridge.cv2_to_imgmsg(image_sized, encoding='mono8'), bridge.cv2_to_imgmsg(image_big,
    #                                                                                encoding='bgr8')


def make_pic(box_set):
    size = 100
    obs = np.zeros((size*10, size*10, 3), dtype=np.uint8)
    # img = np.zeros((1000, 1000, 3), dtype=np.uint8)
    boxes = np.zeros((size*10, size*10), dtype=np.uint8)

    for box in box_set:
        # obs = map_corners(box, size)
        box_polys = map_corners(box, size)
        # cv.fillPoly(img, [box_red], (0, 0, 250))
        cv.fillPoly(boxes, [box_polys], 3)
        cv.fillPoly(obs, [box_polys], 3)
    # print(img.shape)
    obs = view_field(obs, 2, (size*10, size*5), max_distance=500)#max_distance=size*5)
    obs = np.where(obs, np.uint8(255), np.uint8(0))
    print(obs.shape)
    # obs = downsample(obs)
    obssmall = cv.resize(obs, (84, 84), interpolation=cv.INTER_LINEAR)
    cv.imshow('obs', obs)
    cv.imshow('small', obssmall)
    # cv.imshow('boxes', boxes)
    # cv.imwrite('./Desktop/obs{}.jpg'.format(i), img)
    # cv.imwrite('./Desktop/boxes{}.jpg'.format(i), img_boxes)
    cv.waitKey(0)

def map_corners(box, axis):
    new_box = []
    for i in range(4):
        new_corners = []
        # new_corners.append(int((box[i].y * 10 - size*5)*-1)) #x coord, y nfillim pstj x
        # new_corners.append(int((box[i].x * 10 - size*10)*-1))
        new_corners.append(int((box[i][0] - axis) * -1))  # x coord, y nfillim pstj x
        new_corners.append(int((box[i][1] - axis) * -1))
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


def view_field_old(height_map, viewer_height, viewer_pos, max_distance=100, resolution=100): #assume 50m range
    height_map = np.asarray(height_map)
    h, w = height_map.shape
    vi, vj = viewer_pos
    # Find locations higher than viewer
    m = (height_map > viewer_height) & (height_map < 7)
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


def downsample(ar, fact=10):
    assert isinstance(fact, int), type(fact)
    print(fact)
    sx, sy = ar.shape
    x, y = np.ogrid[0:sx, 0:sy]
    regions = sy/fact * (x/fact) + y/fact
    res = ndimage.mean(ar, labels=regions, index=np.arange(regions.max() + 1))
    print(sx/fact)
    res.shape = (np.uint8(sx/fact), np.uint8(sy/fact))
    return res

boxes_example = json.load(open("boxset.json"))
make_pic(boxes_example)