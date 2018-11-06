#!/usr/bin/python
import rospy
import sys
import cv2
from camera.camera import Camera
from threading import Thread
from vision import Vision
from time import time

# Top level imports
import os
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from ROS.ros_vision_publisher import RosVisionService
from utils.model import Model
from utils.camera_loader import CameraLoader
sys.path[0] = old_path
from enum import Enum


class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3
    SET_TEAM_COLOR_BLUE = 4
    SET_TEAM_COLOR_YELLOW = 5

class VisionNode:
    """
    A node for spinning the Vision
    """
    def __init__(self, color=0):
        """
        :param color: int
        """
        self.team_colors = ['blue', 'yellow']
        self.home_color = self.team_colors[color]  # blue or yellow
        self.home_robots = 5
        self.adv_robots = 1
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
        except IndexError:
            model = Model()
            return_type, device = CameraLoader(model.game_opt['camera']).get_index()

        self.camera = Camera(device, root_path + "parameters/CAMERA_ELP-USBFHD01M-SFV.json", threading=False)

        self.vision = Vision(self.camera, self.adv_robots, self.home_color, self.home_robots, self.home_tag,
                             arena_params, colors_params, method="color_segmentation")
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

    def set_team_color(self, color_value):
        """
        Set the vision team color
        :param color_value: int
        :return: nothing
        """
        self.vision.home_color = self.team_colors[color_value-4]  # The value is defined in VisionOperations
                                                                  # This value is also defined in the callback
                                                                  # generated by the interface(4 is blue and 5 for yellow)


if __name__ == "__main__":

    try:
        color = int(sys.argv[2])
    except IndexError:
        color = 1

    vision_node = VisionNode(color)
    rate = rospy.Rate(1)  # 30hz

    while not rospy.is_shutdown():
        if vision_node.show:
            cv2.imshow('vision', cv2.cvtColor(vision_node.vision.arena_image, cv2.COLOR_HSV2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                vision_node.show = not vision_node.show
                cv2.destroyAllWindows()
                vision_node.vision.computed_frames = 0;
                vision_node.vision.t0 = time();
        if vision_node.state_changed:  # Process requisition
            if vision_node.state_changed == VisionOperations.SHOW.value:
                vision_node.show = not vision_node.show

            elif vision_node.state_changed == VisionOperations.CROPPER.value:
                vision_node.vision.params_setter.run()
                vision_node.vision.load_params()

            elif vision_node.state_changed == VisionOperations.COLOR_CALIBRATION.value:
                # This will verify if the color segmentation technique is
                # the chosen one
                if vision_node.vision.colors_params_file != "":
                    # if it is, execute the color calibrator
                    vision_node.vision.color_calibrator.run()
                    vision_node.vision.load_colors_params()

            elif vision_node.state_changed in [VisionOperations.SET_TEAM_COLOR_BLUE.value,
                                               VisionOperations.SET_TEAM_COLOR_YELLOW.value]:
                vision_node.set_team_color(vision_node.state_changed)

            vision_node.state_changed = 0
        rate.sleep()

    vision_node.vision.stop()
    cv2.destroyAllWindows()
