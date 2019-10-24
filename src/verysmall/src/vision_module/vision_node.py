#!/usr/bin/python3
import rospy
import sys
import cv2
from time import time
from enum import Enum
from threading import Thread

from vision_module.camera_module.camera import Camera
from vision_module.vision import Vision
from ROS.ros_vision_publisher import RosVisionService
from utils.model import Model
from utils.camera_loader import CameraLoader


class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3


class VisionNode:
    """
    A node for spinning the Vision
    """

    def __init__(self, vision_owner: str = 'Player_One'):
        """
        :param color: int
        """
        self.team_colors = ['blue', 'yellow']
        self.yellow_robots = 4
        self.blue_robots = 1
        self.show = False
        self.state_changed = 0

        arena_params = "parameters/ARENA.json"
        colors_params = "parameters/COLORS.bin"

        try:
            device = int(sys.argv[1])
        except ValueError:
            device = sys.argv[1]
        except IndexError:
            model = Model()
            _, device = CameraLoader(model.game_opt['camera']).get_index()

        self.camera = Camera(device, "parameters/CAMERA_ELP-USBFHD01M-SFV.bin", threading=False)

        self.vision = Vision(self.camera, self.blue_robots, self.yellow_robots,
                             arena_params, colors_params, method="color_segmentation", vision_owner=vision_owner)
        self.vision.game_on = True

        self.thread = Thread(target=self.vision.run, args=())
        self.thread.daemon = True
        self.thread.start()

        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self.vision_management)

    def vision_management(self, req):
        """
        This is the reading function for a service response
        :param req: variable to get the request operation
        :return: bool
        """
        success = True
        self.state_changed = req.operation
        return success


if __name__ == "__main__":

    try:
        owner_name = sys.argv[2]
    except IndexError:
        owner_name = 1

    vision_node = VisionNode(owner_name)
    rate = rospy.Rate(1)  # 1hz

    while not rospy.is_shutdown():
        if vision_node.show:
            cv2.imshow('vision', cv2.cvtColor(vision_node.vision.arena_image, cv2.COLOR_HSV2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rate = rospy.Rate(1)  # 1hz
                vision_node.show = not vision_node.show
                cv2.destroyAllWindows()
                vision_node.vision.computed_frames = 0
                vision_node.vision.t0 = time()
        if vision_node.state_changed:  # Process requisition
            if vision_node.state_changed == VisionOperations.SHOW.value:
                rate = rospy.Rate(60)  # 60hz
                vision_node.show = True  # not vision_node.show

            elif vision_node.state_changed == VisionOperations.CROPPER.value:
                vision_node.vision.toggle_calibration(True)
                vision_node.vision.params_setter.run()
                vision_node.vision.load_params()
                vision_node.vision.toggle_calibration(False)

            elif vision_node.state_changed == VisionOperations.COLOR_CALIBRATION.value:
                # This will verify if the color segmentation technique is
                # the chosen one
                if vision_node.vision.colors_params_file != "":
                    # if it is, execute the color calibrator
                    vision_node.vision.toggle_calibration(True)
                    vision_node.vision.color_calibrator.run()
                    vision_node.vision.load_colors_params()
                    vision_node.vision.toggle_calibration(False)

            vision_node.state_changed = 0
        rate.sleep()

    vision_node.vision.stop()
    cv2.destroyAllWindows()
