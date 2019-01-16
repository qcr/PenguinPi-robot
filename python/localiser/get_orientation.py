import math
import cv2
import numpy as np
# import socket
import sys

# Image Processing Variables
num_iter = 10
kernel = np.ones((2,2), np.uint8)

# Object Detection
class Box:
    def __init__(self):
        self.x = None
        self.y = None
        self.w = None
        self.h = None

    def __init__(self, xx, yy, ww, hh):
        self.x = xx
        self.y = yy
        self.w = ww
        self.h = hh
        self.cx = xx + (ww/2)
        self.cy = yy + (hh/2)

class Contours:
    def __init__(self):
        self.list_contours = []
        self.list_boxes = []
        self.dec = []
    def get_contours(self, contours):
        minimum_area = 5
        # Find all contours
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            area = w * h
            if area > minimum_area:
                self.list_contours.append(contour)
                self.list_boxes.append(Box(x,y,w,h))



# cam = cv2.VideoCapture(0)


cv2.namedWindow("test")

img_counter = 0

src_points = np.array([[538, 1],[84, 1], [73, 478],[572, 464]])
dst_points = np.array([[0,0],[500,0],[500,500], [0,500]])
h, status = cv2.findHomography(src_points, dst_points)


robot_min = np.array([150, 150, 150])
robot_max = np.array([255, 255, 255])


# im1 = cv2.imread('./partial/10.jpg')
for i in range(0,13):
    im1 = cv2.imread('test0'+ '.jpg')
    # src_points = np.array([[319,24], [1442,37], [1381,1058], [387,1072]])
    #dst_points = np.array([[0,0],[1000,0],[1000,1000], [0,1000]])
    #
    # src_points = np.array([[758, 1], [3075, 1], [639, 2412], [3218, 2424]])
    # dst_points = np.array([[0,0],[1000,0],[0,1000], [1000,1000]])
    # dst_points = np.array([[0,0],[2310,0],[0,2310],[2310,2310]])
    h, status = cv2.findHomography(src_points, dst_points)

    im1 = cv2.warpPerspective(im1, h, (500,500))
    # im_warped_small = cv2.resize(im_out, (500,500))
    #
    # hsv = cv2.cvtColor(im1, cv2.COLOR_BGR2HSV)
    mask_black = cv2.inRange(im1, robot_min, robot_max)
    cv2.imshow("mask", mask_black)
    cv2.waitKey()
    eroded = mask_black
    # dilated = cv2.dilate(mask_black, kernel, iterations = 2)
    # eroded = cv2.erode(dilated, kernel, iterations = 2)
    sm = cv2.resize(eroded, (320,240))

    # cv2.imshow("bw", eroded)
    # cv2.waitKey()

    im2, robot_contours, hierarchy_rbt = cv2.findContours(eroded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("contours", im2)
    cv2.waitKey()

    conts = Contours()
    conts.get_contours(robot_contours)
    boxes = conts.list_boxes
    print("boxes", boxes)
    # print(len(robot_contours))
    for box in boxes:
    # yello
        cv2.rectangle(im1,(box.x,box.y),(box.x+box.w,box.y+box.h),(0,0,255),5)
        print(box.x, box.y)
        # cv2.imshow("test", im1)
        # k = cv2.waitKey(0)



    min_x = 100000
    max_x = 0
    min_y = 100000
    max_y = 0

    # checked = []

    for box in boxes:
        if box.cx < min_x:
            min_x = box.cx

        if box.cx > max_x:
            max_x = box.cx

        if box.cy < min_y:
            min_y = box.cy

        if box.cy > max_y:
            max_y = box.cy

    print(min_x, max_x)
    print(min_y, max_y)

    for box in boxes:
        if box.cx < max_x and box.cx > min_x and box.cy < max_y and box.cy > min_y:
            center_box = box


    dists_boxes = []
    for box2 in boxes:
        if (center_box != box2):
            a = np.array([center_box.cx, center_box.cy])
            b = np.array([box2.cx, box2.cy])
            d = np.linalg.norm(a - b)
            # print(d)
            dist_box = (d, box2)
            dists_boxes.append(dist_box)

    dists_boxes = sorted(dists_boxes, key=lambda x: x[0])
    # print(dists_boxes)
    closest_two = dists_boxes[0:2]
    # print(closest_two)

    mid_point_x = (closest_two[0][1].cx + closest_two[1][1].cx)/2
    mid_point_y = (closest_two[0][1].cy + closest_two[1][1].cy)/2

    angle = np.arctan2(mid_point_x - center_box.cx, mid_point_y - center_box.cy)
    angle = np.rad2deg(angle)
    x = (center_box.cx / 500)*2
    y = (center_box.cy / 500)*2

    print("Pose:")
    print(x)
    print(y)
    print(angle)

    # RED - CENTRE
    cv2.rectangle(im1,(center_box.x,center_box.y),(center_box.x+center_box.w,center_box.y+center_box.h),(0,100,0),7)
    l = 150
    ang = math.radians(-angle-90)
    # print(round(center_box.cx + l * math.cos(ang)), round(center_box.cy + l * math.sin(ang)))
    cv2.arrowedLine(im1, (round(center_box.cx), round(center_box.cy)), (round(center_box.cx + l * math.cos(ang)), round(center_box.cy + l * math.sin(ang))), (0,0,255), 15)
    # cv2.drawContours(im1, robot_contours, -1, (0,255,0), 3)
    # print(len(robot_contours))
    for _,box in closest_two:
        cv2.rectangle(im1,(box.x,box.y),(box.x+box.w,box.y+box.h),(255,100,0),5)



    # small = cv2.resize(im1, (640,480))
    cv2.imshow("test", cv2.resize(im1, (400,400)))

    k = cv2.waitKey(0)

    # cv2.destroyAllWindows()
