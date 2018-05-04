import virtual_field as vf
import cv2 as cv
import numpy as np
import time
from auxiliary import *
from random import uniform


#numbers in centimeters for ball and robots
ball = (uniform(2,148), uniform(2,128)) 
blue1 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecb1 = [uniform(-10,10), uniform(-10,10)]
blue2 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecb2 = [uniform(-10,10), uniform(-10,10)]
blue3 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecb3 = [uniform(-10,10), uniform(-10,10)]
yellow1 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecy1 = [uniform(-10,10), uniform(-10,10)]
yellow2 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecy2 = [uniform(-10,10), uniform(-10,10)]
yellow3 = (uniform(3.5,146.5), uniform(3.5,126.5))
vecy3 = [uniform(-10,10), uniform(-10,10)]

blue = [(blue1,vecb1),(blue2,vecb2),(blue3,vecb3)]
yellow = [(yellow1,vecy1),(yellow2,vecy2),(yellow3,vecy3)]

if __name__ == '__main__':

	virtual = vf.Virtual_Field()		
	
	i = 1	
	
	while True:
		
		virtual.plot_arena()
		virtual.plot_ball(ball)
		virtual.plot_robots(blue, virtual.colors["blue"])
		#virtual.plot_robots(blue2, vecb2, virtual.colors["blue"])
		#irtual.plot_robots(blue3, vecb3, virtual.colors["blue"])
		virtual.plot_robots(yellow, virtual.colors["yellow"])
		#virtual.plot_robots(yellow2, vecy2, virtual.colors["yellow"])
		#virtual.plot_robots(yellow3, vecy3, virtual.colors["yellow"])
		cv.imshow('Campo Virtual', virtual.field)
		
	
		#vec = rotate_vector(vec, 0.1)
		#blue1 =	(blue1[0] + 4, blue1[1] - 4) 
		#ball = (ball[0] - 1, ball[1])

		k = cv.waitKey(1)
		
		if k == ord('q'):
			cv.destroyAllWindows()
			break

		elif k == ord('s'):
			cv.imwrite("frame" + str(i) + ".png", virtual.field)
			i = i+1

		elif k == ord('p'):
			time.sleep(5)

		#elif k == ord('n'):
		ball = (uniform(2,148), uniform(2,128)) 
		blue1 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecb1 = [uniform(-10,10), uniform(-10,10)]
		blue2 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecb2 = [uniform(-10,10), uniform(-10,10)]
		blue3 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecb3 = [uniform(-10,10), uniform(-10,10)]
		yellow1 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecy1 = [uniform(-10,10), uniform(-10,10)]
		yellow2 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecy2 = [uniform(-10,10), uniform(-10,10)]
		yellow3 = (uniform(3.5,146.5), uniform(3.5,126.5))
		vecy3 = [uniform(-10,10), uniform(-10,10)]
		blue[0] = (blue1,vecb1)
		blue[1] = (blue2,vecb2)
		blue[2] = (blue3,vecb3)
		yellow[0] = (yellow1,vecy1)
		yellow[1] = (yellow2,vecy2)
		yellow[2] = (yellow3,vecy3)

		virtual.pause(6)