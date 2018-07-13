import numpy as np
import cv2


class BallSeeker:

    def __init__(self, field_origin, conversion_factor):
        self.field_origin = field_origin
        self.conversion_factor = conversion_factor

    def pixel_to_real_world(self, pos):
        # This function expects that pos is a 1D numpy array

        pos = pos - self.field_origin
        pos[1] *= -1

        return pos * self.conversion_factor

    def seek_ball(self, img, ball):
        cnts = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
        pos = np.array([None, None])

        # Hopefully we have just one contour
        for cnt in cnts:
            # Compute the contour moment to extract its center
            M = cv2.moments(cnt)

            # Calculates the contour center
            if M['m00'] != 0:
                c_x = int(M['m10'] / M['m00'])
                c_y = int(M['m01'] / M['m00'])
                pos = self.pixel_to_real_world(np.array([c_x, c_y]))
                ball.update(-1, pos)
