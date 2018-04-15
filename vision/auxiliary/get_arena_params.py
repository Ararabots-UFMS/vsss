import sys
sys.path.append('../camera')

from camera import Camera
import numpy as np
import cv2
import time

CAMERA_ID = 1
CAMERA_NAME = "ELP-USBFHD01M-SFV"
RED     =   (0, 0, 255)
GREEN   =   (0, 255, 0)
RADIUS  =   3


class CallBackParams:
    def __init__(self):
        self.frame = None
        self.points = []
        self.LBUTTONPRESSED = 0
        self.dragging_point = []

def inside_circle(points, pos, radius):
    i = 0
    for point in points:
        d = np.asarray(point) - np.asarray(pos)
        if np.linalg.norm(d) <= radius:
            return i
        i += 1
    return None

def drawing(p):
    # drawing
    if len(p.points) <= 4:
        for point in p.points:
            cv2.circle(p.frame, point, RADIUS, RED, -1)

    if p.dragging_point != []:
        cv2.circle(p.frame, p.dragging_point[0], RADIUS, GREEN, -1)         
            
    if len(p.points + p.dragging_point) == 4:
        sorted_points = sort_clock_wise(p.points + p.dragging_point)
        pts = sorted_points.reshape((-1, 1, 2))
        frame = cv2.polylines(p.frame, [pts], True, GREEN)

def onMouse(event, x, y, flags, p):
    i = inside_circle(p.points, (x,y), RADIUS)

    if event == cv2.EVENT_LBUTTONDOWN:
        p.LBUTTONPRESSED = 1;

    if event == cv2.EVENT_LBUTTONUP:
        p.LBUTTONPRESSED = 0;
        p.dragging_point = []
        if len(p.points) < 4 and i == None:
            p.points.append((x,y))
        elif i != None:
            del p.points[i]

    if event == cv2.EVENT_MOUSEMOVE and p.LBUTTONPRESSED:
        i = inside_circle(p.points, (x,y), 15*RADIUS)
        if i != None:
            del p.points[i]
        p.dragging_point = [(x,y)]

def sort_clock_wise(points):
    corners = np.asarray(points)
    x_sorted = corners[np.argsort(corners[:, :1].reshape(4)), :]

    left = x_sorted[:2, :]
    right = x_sorted[2:, :]

    (top_left, bottom_left) = left[np.argsort(left[:, 1:].reshape(2)), :]
    (top_right, bottom_right) = right[np.argsort(right[:, 1:].reshape(2)), :]

    return np.array([top_left, top_right, bottom_right, bottom_left])

def get_matrix_transform(pts):
    points = sort_clock_wise(pts)
    (tl, tr, br, bl) = points

    width_a = np.sqrt( (tl[0] - tr[0])**2 + (tl[1] - tr[1])**2 )
    width_b = np.sqrt( (bl[0] - br[0])**2 + (bl[1] - br[1])**2 )
    final_width = max(int(width_a), int(width_b)) - 1

    height_a = np.sqrt( (tl[0] - bl[0])**2 + (tl[1] - bl[1])**2 )
    height_b = np.sqrt( (tr[0] - br[0])**2 + (tr[1] - br[1])**2 )
    final_height = max(int(height_a), int(height_b)) - 1

    dst = np.array([
        [0,0],
        [final_width, 0],
        [final_width, final_height],
        [0, final_height]], dtype="float32")

    return cv2.getPerspectiveTransform(points.astype("float32"), dst), (final_width, final_height)

if __name__ == '__main__':
    # cap = Camera(CAMERA_ID, CAMERA_NAME)
    cap = cv2.VideoCapture(0)
    p = CallBackParams()
    cv2.namedWindow('visualizacao')
    cv2.setMouseCallback('visualizacao', onMouse, p)
    k = 0

    while True:
        ret, p.frame = cap.read()
        key = cv2.waitKey(1) & 0xFF

        if  key == ord('q'):
            print "Exiting"
            break
        elif key == ord('s'):
            if len(p.points) == 4:
                print "Saving points"
                k = 1
                break
            else:
                print "Select at least 4 points"

        drawing(p)
        cv2.imshow('visualizacao', p.frame)
    
    cv2.destroyAllWindows()

    if k:
        print "Calculating prespective transform"
        M, size = get_matrix_transform(p.points)



    while True:
        ret, p.frame = cap.read()
        nframe = cv2.warpPerspective(p.frame, M, (size[0], size[1]))
        cv2.imshow('transformed', nframe)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
