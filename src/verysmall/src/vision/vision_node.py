#!/usr/bin/python
import rospy
import sys
from camera.camera import Camera
from threading import Thread
from vision import Vision

# Top level imports
import os
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
sys.path[0] = old_path

if __name__ == "__main__":

    home_color = "yellow"  # blue or yellow
    home_robots = 5
    adv_robots = 1
    home_tag = "aruco"

    frame_hater = int(1/60*1000)

    arena_params = root_path+"parameters/ARENA.json"
    colors_params = root_path+"parameters/COLORS.json"

    try:
        device = int(sys.argv[1])
    except ValueError:
        device = sys.argv[1]

    camera = Camera(device, root_path+"parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

    v = Vision(camera, adv_robots, home_color, home_robots, home_tag,
               arena_params, colors_params, method="color_segmentation")
    v.game_on = True

    t = Thread(target=v.run, args=())
    t.daemon = True
    t.start()

    rospy.on_shutdown(v.stop)

    rospy.spin()

