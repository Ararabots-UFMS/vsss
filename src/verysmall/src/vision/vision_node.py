#!/usr/bin/python
import rospy
import sys
import cv2
from camera.camera import Camera
from threading import Thread
from vision import Vision
from verysmall.srv import vision_command

# Top level imports
import os
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
sys.path[0] = old_path


class VisionNode:
    def __init__(self):

        self.home_color = "yellow"  # blue or yellow
        self.home_robots = 5
        self.adv_robots = 2
        self.home_tag = "aruco"
        self.show = False
        self.state_changed = 0

        frame_hater = int(1 / 60 * 1000)

        arena_params = root_path + "parameters/ARENA.json"
        colors_params = root_path + "parameters/COLORS.json"

        try:
            device = int(sys.argv[1])
        except ValueError:
            device = sys.argv[1]

        self.camera = Camera(device, root_path + "parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=True)

        self.vision = Vision(self.camera, self.adv_robots, self.home_color, self.home_robots, self.home_tag,
                             arena_params, colors_params, method="color_segmentation")
        self.vision.game_on = True

        self.thread = Thread(target=self.vision.run, args=())
        self.thread.daemon = True
        self.thread.start()

        # Creates the service responsible for vision modes and operations
        self.service = rospy.Service('vision_command', vision_command, self.vision_management)

    def vision_management(self, req):
        success = True
        self.state_changed = req.operation
        return success


if __name__ == "__main__":

    vision_node = VisionNode()
    rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():
        if vision_node.show:
            cv2.imshow('vision', vision_node.vision.adv_seg)
            cv2.waitKey(1) & 0xFF
        if vision_node.state_changed:  # Process requisition
            if vision_node.state_changed == 1:
                vision_node.show = not vision_node.show
                if not vision_node.show:
                    cv2.destroyWindow("vision")
                vision_node.state_changed = 0

            elif vision_node.state_changed == 2:
                vision_node.vision.params_setter.run()
                vision_node.vision.load_params()
                vision_node.state_changed = 0
        rate.sleep()

    vision_node.vision.stop()
    cv2.destroyAllWindows()

