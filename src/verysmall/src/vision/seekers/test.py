from simple_object_detector import SimpleObjectDetector
from aruco_object_detector import ArucoObjectDetector
from kmeans_object_detector import KmeansObjectDetector
import cv2
import numpy as np

if __name__ == '__main__':
	img = np.zeros((512,512,3), np.uint8)
	cv2.rectangle(img,(50,50),(100,100),(255,0,0),-1)
	cv2.rectangle(img,(200,200),(250,250),(255,0,0),-1)
	cv2.imshow("a",img)
	general = SimpleObjectDetector()
	kmeans = KmeansObjectDetector()
	#aruco = ArucoObjectDetector()

	general.seek([img],[2])

	key = None
	
	while key != ord('q'):
		cv2.imshow("a",img)
		key = cv2.waitKey(0)