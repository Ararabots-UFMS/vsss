import numpy as np
import cv2
import cv2.aruco as aruco
import math

from vision_module.seekers.seeker_data_structures import *
from vision_module.seekers.obj_detector import ObjDetector

# @author Wellington Castro <wvmcastro>


class ArucoObjectDetector(ObjDetector):

    def __init__(self, cam_mtx, dist_vec, num_tags, num_bits=3, num_markers=5):
        """ Initializes the objects necessary to perform the detection
            of the aruco tags """

        super().__init__()

        # Hyper params to create the aruco markers dictionary
        # self.num_markers = num_markers
        self.num_markers = 68
        self.num_bits = num_bits

        # This variable will keep how many tags must be identified
        self.num_tags = num_tags

        # Creates the Aruco dictionary according to the number of bits and
        # number of desired markers
        self.aruco_dict = aruco.Dictionary_create(self.num_markers, self.num_bits)
        self.aruco_params = aruco.DetectorParameters_create()

        # Important info from the camera needed by the ArucoDetectMarkers
        self.camera_matrix = cam_mtx
        self.distortion_vector = dist_vec

        self.obj_size = -1



    def get_aruco_state(self, img, rvec, tvec, degree=False):
        # x axis
        x_axis = np.float32([[0,0,0],[1.0,0,0]]).reshape(-1,3)

        # Do not know what is jac (sorry again). Imgpts are the start and end points
        # of each vector of the base
        imgpts, jac = cv2.projectPoints(x_axis, rvec, tvec, self.camera_matrix, self.distortion_vector)

        # takes the tail and nose of the orientation vector
        t, n = imgpts[0][0], imgpts[1][0]

        # calculates the orientation vector
        orientation_vec = (n-t)
        orientation_vec[1] *= -1
        orientation_angle = math.atan2(orientation_vec[1], orientation_vec[0])

        if degree == True:
            #TODO:
            pass
            #orientation_angle *= self.rad_to_degree_factor

        # returns the center of the aruco marker and its x axis orientation
        return t, orientation_angle

    def update_obj_size(self, obj_rectangle):
        tl = Vec2(np.inf, np.inf)
        br = Vec2(-np.inf, -np.inf)
        for corner in obj_rectangle:
            tl.x = min(tl.x, corner[0])
            tl.y = min(tl.y, corner[1])
            br.x = max(br.x, corner[0])
            br.y = max(br.y, corner[1])

        wh = br - tl
        self.obj_size = abs(wh)

    def aruco_seek(self, img, number_of_tags=None, degree=False):
        """
        Receives an image and the number of tags in it, the applies the aruco seeker to identify tags and returns
        tags from segment.
        :param img: Segment of image
        :param degree: if orientation is returned in degrees
        :param number_of_tags: number of tags ins segment
        :return: Array(ObjState)
        """
        if number_of_tags is None:
            number_of_tags = self.num_tags

        # Try to locate all markers in the img
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        identified_markers = []

        if np.any(ids is not None):
            # That means at least one Aruco marker was recognized

            # Reshapes the ids matrix to an ids vector for indexing simplicity
            ids = ids.reshape(ids.shape[0] * ids.shape[1])

            # Sort the ids vector, that way the same marker will be always in the
            # same pos in the things_list
            sorted_ids = np.sort(ids)

            i = 0
            # rospy.logfatal('ids.size: '+str(ids.size))
            # rospy.logfatal('sorted[i]: ' + str(sorted_ids[i]))

            while(i < ids.size and i < number_of_tags):

                # Finds the original index of the marker before sorting
                index = np.argwhere(ids == sorted_ids[i])[0,0]

                if self.should_calculate_size:
                    self.update_obj_size(corners[index][0])

                # Estimate the marker's pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[ index ], 0.075,
                                self.camera_matrix, self.distortion_vector)

                # Gets the marker state, ie: its center and x axis orientation
                center, orientation = self.get_aruco_state(img, rvec, tvec, degree)

                obj_state = ObjState()
                obj_state.id = sorted_ids[i]
                obj_state.set_pos(center[0], center[1])
                obj_state.set_orientation(orientation)

                identified_markers.append(obj_state)

                i += 1

        #print("FOUND:", identified_markers)
        return identified_markers

    def seek(self, segments, objects_per_segment, full_image):
        """
            This function receives a list of binary images and list of number of objects per segment
            and return its centers positions per segment using a opencv aruco implementation
            :param segments: np.array([uint8]).shape([m,n])
            :param objects_per_segment: np.array
            :return: np.array([ObjState]).shape([k, n]) object has the position of the center of each object in img
        """

        # Our return value
        centroids_per_segment = []
        obj_counter = 0
        number_of_segments = len(segments)

        # Iterate over segments
        for index in range(number_of_segments):
            # Invert image colors so we can see with aruco detector
            centroids_per_segment.append(self.aruco_seek(img=segments[index], number_of_tags=objects_per_segment[index]))
            obj_counter += len(centroids_per_segment)

        if obj_counter < sum(objects_per_segment)
        return centroids_per_segment
