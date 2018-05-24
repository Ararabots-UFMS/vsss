

import cv2
import numpy as np

global img

# mouse callback function
def draw_circle(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		#print frame[x][y]
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		color = [hsv[y][x][0],hsv[y][x][1],hsv[y][x][2]]
		lists.append(color)
		lists.sort()

		print lists
		print lists[0], lists[len(lists)-1]
		

		low = np.array(lists[0])
		high = np.array(lists[len(lists)-1])

		img = cv2.inRange(hsv, low, high)
		cv2.imshow('mask',img)

		
			#img[y][x] = [255,255,255]


# Create a black image, a window and bind the function to window

img = np.zeros((480,640,3), np.uint8)
cv2.namedWindow('camera')
cv2.namedWindow('mask')
cv2.setMouseCallback('camera',draw_circle)
cam = cv2.VideoCapture(0)
cam.set(4,640)
cam.set(3,480)

lists = []

while(1):

	ret, frame = cam.read()
	framer = frame
	cv2.imshow('camera', frame)
	

	if cv2.waitKey(20) & 0xFF == ord('q'):
		break
cv2.destroyAllWindows()

