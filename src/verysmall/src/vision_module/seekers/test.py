# from simple_object_detector import SimpleObjectDetector
# from aruco_object_detector import ArucoObjectDetector
# from kmeans_object_detector import KmeansObjectDetector
# import cv2
# import numpy as np
#
# if __name__ == '__main__':
# 	img = np.zeros((512,512,3), np.uint8)
# 	cv2.rectangle(img,(50,50),(100,100),(255,0,0),-1)
# 	cv2.rectangle(img,(200,200),(250,250),(255,0,0),-1)
# 	cv2.imshow("a",img)
# 	general = SimpleObjectDetector()
# 	kmeans = KmeansObjectDetector()
# 	#aruco = ArucoObjectDetector()
#
# 	general.seek([img],[2])
#
# 	key = None
#
# 	while key != ord('q'):
# 		cv2.imshow("a",img)
# 		key = cv2.waitKey(0)
from vision_module.seekers.new_seeker import NewSeeker
from vision_module.seekers.seeker_data_structures import Vec2
from vision_module.seekers.seeker_data_structures import ObjState

if __name__ == '__main__':
	n = 3
	objs_pos = [(2,2), (1,1), (6,1)]
	objs = [ObjState() for k in range(n)]
	for i in range(n):
		objs[i].set_pos(*objs_pos[i])

	seeker = NewSeeker(n, None)
	trackers_pos = [(3,3), (1,6), (6,2)]
	for i in range(n):
		seeker.trackers[i].set_pos(*trackers_pos[i])

	trackers_index = [i for i in range(n)]

	a = seeker.cluster_objects_and_trackers(trackers_index, objs)
	print([o.pos for o in a])
