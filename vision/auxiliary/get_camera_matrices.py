#!/usr/bin/python
import numpy as np
import glob
import cv2
import time
import sys
import json
import os
sys.path.append('../camera')
from camera import Camera

CAMERA_ID = 1
CAMERA_NAME = ""
FRAME_SIZE = () #(width, height)
SCALE_FACTOR = 1.0
H_CENTERS = 12
V_CENTERS = 9

def get_error(objp, imgpoints, rvecs, tvecs, mtx, dist):
    mean_error = 0
    for i in xrange(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    return mean_error/len(objpoints)

if __name__ == '__main__':
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((H_CENTERS * V_CENTERS,3), np.float32)
    objp[:,:2] = np.mgrid[0:H_CENTERS, 0:V_CENTERS].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('*.jpg')

    for fname in images:
        frame = cv2.imread(fname)
        
        # scaling
        (w,h) = frame.shape[:2]
        frame = cv2.resize(frame, (int(SCALE_FACTOR * w), int(SCALE_FACTOR * h)), interpolation = cv2.INTER_LINEAR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Find the chessboard centers
        ret, corners = cv2.findChessboardCorners(gray, (H_CENTERS, V_CENTERS), None)

        if ret:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(frame, (H_CENTERS, V_CENTERS), corners,ret)
        
        cv2.imshow('window', frame)
        cv2.waitKey(100)


    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    h, w =  frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # get undistorion maps
    mapx,mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx,( w,h), 5)

    e = get_error(objp, imgpoints, rvecs, tvecs, mtx, dist)
    print "STD Error:", e

    save = raw_input("Save camera matrices? (y/n): ")
    print save

    if save == 'y':
        if CAMERA_NAME == "":
            name = raw_input("Please insert camera name: ")
        else:
            name = CAMERA_NAME
        if FRAME_SIZE == ():
            frame_width = raw_input("Please insert frame width: ")
            frame_height = raw_input("Please insert frame height: ")
        else:
            frame_width = FRAME_SIZE[0]
            frame_height = FRAME_SIZE[1]

        file_name = "../../parameters/CAMERA_"+name+".json"
        params = {}
        params['matrix_x'] = mapx.tolist()
        params['matrix_y'] = mapy.tolist()
        params['default_frame_width'] = int(frame_width)
        params['default_frame_height'] = int(frame_height)
        file = open(file_name, "w+")
        json.dump(params, file)
        file.close()



    print "Showing result, press q to exit"
    if save == 'y':
        cap = Camera(CAMERA_ID, name)
    else:
        cap = Camera(CAMERA_ID, "", False, False)

    while(True):
        u_frame = cap.read()
        if save != 'y':
            u_frame = cv2.remap(frame,mapx,mapy,cv2.INTER_LINEAR)    
        
        cv2.imshow('Undistorted', u_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    os.system("rm *.jpg")
    print "all .jpg files removed!"
    cv2.destroyAllWindows()
