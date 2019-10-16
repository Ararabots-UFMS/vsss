from vision_module.camera_module.camera import Camera
import time
import numpy as np
import uuid
import cv2
import sys

CAMERA_ID = 0
PARAMS_FILE = ""
FRAME_SIZE = (1280, 720)

if __name__ == '__main__':
    CAMERA_ID = int(sys.argv[1])
    cap = Camera(CAMERA_ID, PARAMS_FILE, False)
    if FRAME_SIZE != ():
        cap.set_device(FRAME_SIZE[0], FRAME_SIZE[1])

    while(True):
        _, frame = cap.read()
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            name = uuid.uuid4().hex + ".jpg"
            cv2.imwrite(name, frame)
            print("Frame saved")
        elif key == ord('q'):
            print("Exiting")
            break

cv2.destroyAllWindows()
