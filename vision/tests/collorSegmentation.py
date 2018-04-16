import cv2
import numpy as np

# mouse callback function
def getXY(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		#print type(mask[y,x])
		mask[y,x] = [255,255,255]
		mask[y-1,x] = [255,255,255]
		mask[y+1,x] = [255,255,255]
		mask[y,x-1] = [255,255,255]
		mask[y,x+1] = [255,255,255]
		mask[y-1,x-1] = [255,255,255]
		mask[y+1,x+1] = [255,255,255]
		mask[y-1,x+1] = [255,255,255]
		mask[y+1,x-1] = [255,255,255]
		
		cv2.imshow('mask',mask) 



if __name__ == '__main__':

	cam = cv2.VideoCapture(1)
	ret, img =  cam.read()

	Y_DIMENSION, X_DIMENSION, CH  = img.shape
	mask = np.zeros(shape=(Y_DIMENSION,X_DIMENSION,3))

	# Create a black image, a window and bind the function to window
	#img = np.zeros((512,512,3), np.uint8)
	cv2.namedWindow('image')
	cv2.namedWindow('mask')
	cv2.imshow('mask',mask)

	cv2.setMouseCallback('image',getXY, mask )

	pixels = []

	while(1):
		ret, img = cam.read()
		cv2.imshow('image',img)
		if cv2.waitKey(20) & 0xFF == 27:
			break
	cv2.destroyAllWindows()
