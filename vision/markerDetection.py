import cv2
import numpy as np


aruco = cv2.aruco

def main():
	#correct here
	detector = aruco.marker()
	params = detector.getParameters()

	frame = imread('/Pictures/teste.png')

	markers = detector.detect(frame)

	for marker in markers:
		markers.draw(frame, np.array([255,0,0]), 2)

	cv2.imshow('Marker Detection', frame)
	cv2.waitKey(100)


if __name__ == '__main__':
	main()