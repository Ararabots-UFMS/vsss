import virtual_field as vf
from auxiliary import *
import cv2 as cv
import numpy as np
import time
from random import uniform

#example of how to use the virtual_field class



#gerenation of objects, must be changed for ROS update os values 
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
		virtual.plot_robots(yellow, virtual.colors["yellow"])
		
		cv.imshow('Campo Virtual', virtual.field)
		
	
	
		k = cv.waitKey(1)
		
		if k == ord('q'):
			cv.destroyAllWindows()
			break

		elif k == ord('s'):
			cv.imwrite("frame" + str(i) + ".png", virtual.field)
			i = i+1

		elif k == ord('p'):
			print(type(virtual.field))
			time.sleep(5)




		#update of objects, must be changed for ROS update os values 
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

		virtual.pause(5)