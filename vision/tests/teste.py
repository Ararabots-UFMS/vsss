import virtual_field as vf
import cv2 as cv
import numpy as np
import time

ball = (300, 500)
blue1 = (200, 200)
vec = [-5,-10]
vec2 = [3, 10]
yellow1 = (384, 347)

if __name__ == '__main__':

	virtual = vf.Virtual_Field()		
	print virtual.field_origin

	while 1:
		virtual.plot_arena()

		virtual.plot_ball(ball)
		virtual.plot_robots(blue1, vec, virtual.blue)
		virtual.plot_robots(yellow1, vec2, virtual.yellow)
		cv.imshow('Campo Virtual', virtual.field)
		
		

		k = cv.waitKey(0)
		if k == ord('q'):
			cv.destroyAllWindows()
			break

		elif k == ord('s'):
			cv.imwrite("frame.png", virtual.field)