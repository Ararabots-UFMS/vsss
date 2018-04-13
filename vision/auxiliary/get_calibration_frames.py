import sys
sys.path.append('../camera')

from camera import Camera
import time
import numpy as np
import uuid
import cv2

CAMERA_ID = 0
CAMERA_NAME = ""
FRAME_SIZE = ()

if __name__ == '__main__':
    cap = Camera(CAMERA_ID, CAMERA_NAME, False, False)
    if FRAME_SIZE != ():
        cap.set_frame_size(FRAME_SIZE[0], FRAME_SIZE[1])

    while(True):
        frame = cap.read()

        cv2.imshow("get frames window", frame)

        if cv2.waitKey(1) & 0xFF == ord(' '):
            name = uuid.uuid4().hex + ".jpg"
            cv2.imwrite(name, frame)
            print "Frame saved"

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print "Exiting"
            break

cv2.destroyAllWindows()