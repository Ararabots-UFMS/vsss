import numpy as np
import cv2
from seeker_data_structures import *


class SimpleObjectDetector():
    """
        A simple detector using find contours
        :param : None
    """
    def __init__(self):
        self.obj_size = -1
        self.__cnts = None
        pass

    def update_obj_size(self):
        *_, w, h = cv2.boudingRect(self.__cnts)
        self.obj_size = max(max(w,h), self.obj_size)

    def seek(self, segments, objects_per_segment):
        """
            This function receives a list of binary images and list of number of objects per segment
            and return its centers positions per segment using a simple opencv find contours function
            :param segments: np.array([uint8]).shape([m,n])
            :param objects_per_segment: np.array
            :return: np.array([float, float]).shape([k, 2]) object has the position of the center of each object in img
        """

        # Our return value
        centroids_per_segment = np.array([])

        number_of_segments = len(segments)

        # Iterate over segments
        for index in xrange(number_of_segments):

            self.__cnts = cv2.findContours(segments[index], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]

            c_x, c_y = None, None

            obj_states_in_this_segment = []

            for cnt in self.__cnts:
                # Compute the contour moment to extract its center
                M = cv2.moments(cnt)

                # Calculates the contour center
                if M['m00'] != 0:
                    c_x = int(M['m10'] / M['m00'])
                    c_y = int(M['m01'] / M['m00'])

                temp_obj_state = ObjState()
                temp_obj_state.set_pos(c_x, c_y)

                obj_states_in_this_segment.append(temp_obj_state)
                self.update_obj_size()

        centroids_per_segment = np.append(centroids_per_segment, obj_states_in_this_segment)

        return centroids_per_segment
