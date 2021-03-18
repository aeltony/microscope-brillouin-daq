# Run mako_pupil.py to see just the algorithm in action from a live camera

import cv2
import numpy as np
import math
from time import sleep

class PupilDetection:

	def __init__(self):

		self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
		# define pupil detection constants here

	# imageData is a numpy ndarray
	# TODO: clean up constants, like the contour area discriminator, circularity etc
	def DetectPupil(self, imageData, radiusGuess):

		#add color to image for colored pupil ring
		image = cv2.cvtColor(imageData, cv2.COLOR_GRAY2BGR)
		#make image grayscale for image processing 
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# grayBlur = cv2.medianBlur(gray,5)
		grayBlur = cv2.blur(gray,(5,5))
		"""
		THRESHOLD, used to differentiate dark and light colors, FINDS PUPIL
		First number is used for threshold of dark items - the smaller the number, 
		the darker the item must be. Numbers can vary based on level the LED is set 
		too.At first notch on LED controller, Left eye - 25 is best value; Right eye - 23 
		Works best. This difference is due too the right eye not being illuminated as 
		well as the right eye - maybe due too the angle of the LED, or the nose blocking 
		some of the light. When the right eye looks left or right,  the iris is not lit 
		up enough, so the algorithm circles it as it cannot differentiate it from the pupil. 
		This is why is requires a smaller threshold (darker).
		"""
		pupilFrac = math.pi*radiusGuess*radiusGuess/1000./1000. # Approx. fraction of image taken up by pupil

		# print('np.mean(gray) =', np.mean(grayBlur))
		# print('np.min(gray) =', np.amin(grayBlur))
		# print('np.quantile(grayBlur, pupilFrac) =', np.quantile(grayBlur, pupilFrac))
		# print('np.quantile(grayBlur, 0.5) =', np.quantile(grayBlur, 0.5))
		retval, threshold = cv2.threshold(grayBlur, np.quantile(grayBlur, pupilFrac)+5, 255, 0)

		#cv2.imshow("threshold", threshold)

		#cleans up threshold image
		# closed = threshold
		closed = cv2.erode(cv2.dilate(threshold, self.kernel, iterations=1), self.kernel, iterations=1)

		#cv2.imshow("closed", closed)


		threshold, contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

		closed = cv2.cvtColor(closed, cv2.COLOR_GRAY2BGR)
		 ###### for recording video

		drawing = np.copy(image)
		center = (np.nan, np.nan)
		#draws contours on video
		# cv2.drawContours(drawing, contours, -1, (255, 0, 0), 2) 

		# print("[DetectPupil] num contours: %d" % len(contours))

		for contour in contours:

			contour = cv2.convexHull(contour)
			area = cv2.contourArea(contour)

			#SIZE of pupil ranges, can  adjust to larger or smaller values 
			#used to get rid of smaller or larger detections
			#lower bound ~3000, works better in dark room - more dialated pupil 
			if area < 0.25*math.pi*radiusGuess*radiusGuess or area > 4*math.pi*radiusGuess*radiusGuess:
			    continue

			#focuses on rounder shapes 
			circumference = cv2.arcLength(contour, True)
			circularity = circumference**2 / (4*math.pi*area)
			#closer to 1, the more circluar the elipical shapes
			if circularity > 1.2:
			    continue

			#print area
			bounding_box = cv2.boundingRect(contour)

			extend = area / (bounding_box[2] * bounding_box[3])

			# reject the contours with big extend
			if extend > 0.8:
			    continue

			# calculate countour center and draw a dot there
			m = cv2.moments(contour)
			if m['m00'] != 0:
			    dot = (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
			    cv2.circle(drawing, dot, 4, (0, 255, 0), -1)
			    center = (int(m['m01'] / m['m00']), int(m['m10'] / m['m00']))

			# fit an ellipse around the contour and draw it into the image
			try:
			    ellipse = cv2.fitEllipse(contour)
			    cv2.ellipse(drawing, box=ellipse, color=(0, 255, 0), thickness = 4)
			    # 'Pupil radius =', np.sqrt(area/math.pi)
			except:
			    pass

			return (drawing, center)

		# print('Pupil center =', center)
		return (drawing, center)

if __name__ == "__main__":
	testImage = 128 * np.ones((512,512, 3))
	PD = PupilDetection()
	res = PD.DetectPupil(testImage)
	print(res)
	print("done")