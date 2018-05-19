import sys
sys.path.append('../')
sys.path.append('../../')

from camera.camera import Camera
import COLORS
import numpy as np
import cv2
import time
import os
from json_handler import JsonHandler

# modes definitions
NO_MODE     = -1
ADD_MODE    = 0
DELETE_MODE = 1
EDIT_MODE   = 2

CAMERA_ID = 1
CAMERA_PARAMS_PATH = "../../parameters/CAMERA_ELP-USBFHD01M-SFV.json"
ARENA_PARAMS_PATH = ""
RADIUS  =   3
BAR_HEIGHT = 50
MOUSE_LAST_STATE = None

def get_status_bar(h, w, status):
    font = cv2.FONT_HERSHEY_SIMPLEX
    status_bar = np.zeros((h, w, 3), np.uint8)

    if status == ADD_MODE:
        text = "ADD POINT MODE"
        color = COLORS.GREEN
    elif status == DELETE_MODE:
        text = "DELETE POINT MODE"
        color = COLORS.RED
    elif status == EDIT_MODE:
        text = "EDIT POINT MODE"
        color = COLORS.YELLOW
    elif status == NO_MODE:
        text = "NO MODE SELECTED"
        color = COLORS.BLUE

    status_bar[:][:] = color
    return cv2.putText(status_bar, text, (0,30), font, 1, (0, 0, 0), 2, cv2.LINE_AA)

def inside_circle(points, pos, radius):
    i = 0
    for point in points:
        d = np.asarray(point) - np.asarray(pos)
        if np.linalg.norm(d) <= radius:
            return i
        i += 1
    return None

def draw_components(img, pts, sort=True):
    if len(pts):
        if sort:
            sorted_points = sort_clockwise(pts)
        else:
            sorted_points = pts

        for (i,pt) in enumerate(sorted_points):
            cv2.circle(img, tuple(pt), RADIUS, COLORS.RED, -1)
            if i != 0:
                segment = np.array([sorted_points[j], sorted_points[i]]).reshape((-1, 1, 2))
                cv2.polylines(img, [segment], True, COLORS.GREEN, 2)
            j = i

        if len(pts):
            segment = np.array([sorted_points[j], sorted_points[0]]).reshape((-1, 1, 2))
            cv2.polylines(img, [segment], True, COLORS.GREEN, 2)

def onMouse_add_mode(event, x, y, flags, pts):
    if event == cv2.EVENT_LBUTTONUP:
        pts.append((x,y-BAR_HEIGHT))

def onMouse_delete_mode(event, x, y, flags, pts):
    i = inside_circle(pts, (x,y-BAR_HEIGHT), RADIUS)
    if event == cv2.EVENT_LBUTTONUP:
        if i != None:
            del pts[i]

def onMouse_edit_mode(event, x, y, flags, pts):
    i = inside_circle(pts, (x,y-BAR_HEIGHT), 10*RADIUS)
    global MOUSE_LAST_STATE # sorry

    if event == cv2.EVENT_LBUTTONDOWN:
        MOUSE_LAST_STATE = cv2.EVENT_LBUTTONDOWN
    elif event == cv2.EVENT_LBUTTONUP:
        MOUSE_LAST_STATE = cv2.EVENT_LBUTTONUP

    if event == cv2.EVENT_MOUSEMOVE and MOUSE_LAST_STATE == cv2.EVENT_LBUTTONDOWN and i != None:
        pts[i] = (x,y-BAR_HEIGHT)

def onMouse_no_mode(event, x, y, flags, pts):
    pass

def sort_clockwise(pts):
    n = len(pts)
    if n > 1:
        points = np.asarray(pts).reshape(n, 2)
        
        points[:, 1:] = -points[:, 1:] 
        tl_index = np.argsort(np.linalg.norm(points, axis=1))[0]
        tl = points[tl_index]
        d = points - tl
        thetas = np.arctan2(d[:, 1:], d[:, :1]).reshape(n)
        thetas[tl_index] = 10 # this way top left will always be the first
        order =  np.argsort(-thetas)

        points[:, 1:] = -points[:, 1:]
        return points[order, ]
    return pts

def get_matrix_transform(pts):
    points = sort_clockwise(pts)
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

def save_params(matrix, vertices, size, path):
    params = {}
    params['warp_matrix'] = matrix.tolist()
    params['arena_vertices'] = vertices
    params['arena_size'] = (size[0], size[1])
    json_handler = JsonHandler()
    return json_handler.write(params, path)

if __name__ == '__main__':
    print "-------------------------------------------"
    print "A to enter Add point mode"
    print "E to enter Edit point mode"
    print "D to enter Delete point mode"
    print "ESC to exit mode selection"
    print "S to save"
    print "Q to quit"
    print "--------------------------------------------\n"
    cap = Camera(0, CAMERA_PARAMS_PATH)
    points = []
    frame = cap.read()
    

    cv2.namedWindow('cropper')
    mode = NO_MODE
    warped = False
    arena_countour = False
    while True:
        frame = cap.read()
        
        if warped:
            frame = cv2.warpPerspective(frame, M, (size[0], size[1]))

        h,w = frame.shape[:2]

        status_bar = get_status_bar(BAR_HEIGHT, w, mode)

        if not warped:
            sort_clockwise(points)
            draw_components(frame, points)
        else:
            draw_components(frame, points, sort=False)

        cv2.imshow('cropper', np.vstack([status_bar, frame]))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            mode = ADD_MODE
            cv2.setMouseCallback('cropper', onMouse_add_mode, points)
        elif key == ord('d'):
            mode = DELETE_MODE
            cv2.setMouseCallback('cropper', onMouse_delete_mode, points)
        elif key == ord('e'):
            mode = EDIT_MODE
            cv2.setMouseCallback('cropper', onMouse_edit_mode, points)
        elif key == ord('s'):
            if warped:
                arena_countour = True
                break
            else:
                if len(points) == 4:
                    warped = True
                    print "Calculating prespective transform"
                    M, size = get_matrix_transform(points)
                    points = []
                    mode = NO_MODE
                    print "Now draw the contours of the arena"
                else:
                    print "Select four points"
        elif key == 27:
            mode = NO_MODE
            cv2.setMouseCallback('cropper', onMouse_no_mode, points)
    cv2.destroyAllWindows()

    if warped and arena_countour:
        if ARENA_PARAMS_PATH == "":
            ARENA_PARAMS_PATH = raw_input("Type the path where the parameters should be shaved (ex: ~/Documents/params.json): ")
            ret = save_params(M, points, size, ARENA_PARAMS_PATH)
            if ret:
                print "Parameters saved!"
