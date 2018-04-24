import plotter as plt
import cv2 as cv
import numpy as np
import time

diff = 30
ball = (340, 260)
blue_1 = 300
blue_2 = 100
contours = [np.array([(blue_1,blue_1),(blue_1,blue_1+diff),(blue_1+diff,blue_1+diff),(blue_1+diff,blue_1)]), np.array([(blue_2,blue_2),(blue_2,blue_2+diff),(blue_2+diff,blue_2+diff),(blue_2+diff,blue_2)])]
contours2 = [np.array([(110,420),(110,420+diff),(110+diff,420+diff),(110+diff,420)])]

blue = [255,0,0]
yellow = [0,255,255]
orange = [0,blue_2,255]

if __name__ == '__main__':

	plot = plt.Plotter('vsss-field-600x520.jpg')

	while 1:

		plot.plot_ball(ball, orange)
		plot.plot_robots(contours, blue)

		plot.plot_robots(contours2, yellow)

		cv.imshow('test', plot.field)


		blue_1 = blue_1 + 1
		blue_2 = blue_2 + 1
		contours = [np.array([(blue_1,blue_1),(blue_1,blue_1+diff),(blue_1+diff,blue_1+diff),(blue_1+diff,blue_1)]), np.array([(blue_2,blue_2),(blue_2,blue_2+diff),(blue_2+diff,blue_2+diff),(blue_2+diff,blue_2)])]

		plot.clean()

		time.sleep(0.016)

		k = cv.waitKey(1) & 0xFF
		if k == ord('q'):
			break