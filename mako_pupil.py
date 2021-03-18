
from pymba import *
import time
import cv2
import numpy as np
import math
import time
"""
Uses pymba which is a Python wrapper for Allied Vision Vimba C API. 
Located at https://github.com/morefigs/pymba, which has a simple example.
Vimba C API documentation is located locally at
C:\\Program Files\\Allied Vision\\Vimba_2.1\\VimbaC\\Documentation

"""

#used when cleaning up threshold algorithm 
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# capture frames from the camera
#start accessing camera software
vimba = Vimba()
vimba.startup()
system = vimba.getSystem()
if system.GeVTLIsPresent:
    system.runFeatureCommand("GeVDiscoveryAllOnce")
    time.sleep(0.2)
camera_ids = vimba.getCameraIds()
print(camera_ids)

#access camera
camera = vimba.getCamera(camera_ids[0])
camera.openCamera()

camera_feature_names = camera.getFeatureNames()

# set the value of a feature
camera.AcquisitionMode = 'Continuous'

# create new frames for the camera
frame = camera.getFrame()  

# announce frame
frame.announceFrame()

# capture a camera image
camera.startCapture()
camera.runFeatureCommand('AcquisitionStart')
frame.queueFrameCapture()

#start recording video
# fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
# output = cv2.VideoWriter('final_circle2.avi', fourcc, 4.85, (2048, 2048))
# output_bw = cv2.VideoWriter('final_bw2.avi', fourcc, 4.85, (2048, 2048))

frames = 0
start = time.time()
while 1:
    
    frames += 1
    frame.waitFrameCapture(1000)
    frame.queueFrameCapture()
    # get image data...
    imgData = frame.getBufferByteData()
    #convervt to numpy array for easier use
    image = np.ndarray(buffer = imgData,
                       dtype = np.uint8,
                       shape = (frame.height,frame.width))      

    #add color to image for colored pupil ring
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    #make image grayscale for image processing 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    """T
    HRESHOLD, used to differentiate dark and light colors, FINDS PUPIL
    First number is used for threshold of dark items - the smaller the number, 
    the darker the item must be. Numbers can vary based on level the LED is set 
    too.At first notch on LED controller, Left eye - 25 is best value; Right eye - 23 
    Works best. This difference is due too the right eye not being illuminated as 
    well as the right eye - maybe due too the angle of the LED, or the nose blocking 
    some of the light. When the right eye looks left or right,  the iris is not lit 
    up enough, so the algorithm circles it as it cannot differentiate it from the pupil. 
    This is why is requires a smaller threshold (darker).
    """
    retval, threshold = cv2.threshold(gray, 25, 255, 0)
 

    cv2.imshow("threshold", threshold)

    #cleans up threshold image
    closed = cv2.erode(cv2.dilate(threshold, kernel, iterations=1), kernel, iterations=1)

    cv2.imshow("closed", closed)


    threshold, contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    closed = cv2.cvtColor(closed, cv2.COLOR_GRAY2BGR)
     ###### for recording video

    drawing = np.copy(image)
    #draws contours on video
    # cv2.drawContours(drawing, contours, -1, (255, 0, 0), 2) 

    for contour in contours:
        contour = cv2.convexHull(contour)
        area = cv2.contourArea(contour)

        #SIZE of pupil ranges, can  adjust to larger or smaller values 
        #used to get rid of smaller or larger detections
        #lower bound ~3000, works better in dark room - more dialated pupil 
        if area < 3000 or area > 100000:
            continue
        
        #focuses on rounder shapes 
        circumference = cv2.arcLength(contour, True)
        circularity = circumference**2 / (4*math.pi*area)
        #closer to 1, the more circluar the elipical shapes
        if circularity > 1.2:
            continue

        print(area)
        bounding_box = cv2.boundingRect(contour)

        extend = area / (bounding_box[2] * bounding_box[3])

        # reject the contours with big extend
        if extend > 0.8:
            continue

        # calculate countour center and draw a dot there
        m = cv2.moments(contour)
        if m['m00'] != 0:
            center = (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
            cv2.circle(drawing, center, 3, (0, 255, 0), -1) 

        # fit an ellipse around the contour and draw it into the image
        try:
            ellipse = cv2.fitEllipse(contour)
            cv2.ellipse(drawing, box=ellipse, color=(0, 255, 0), thickness = 3)
        except:
            pass


    # output.write(drawing) ##### for recording video
    # show the frame
    cv2.imshow("Drawing", drawing)
    k = cv2.waitKey(1)
    #press Esc key to exit video window
    if k == 0x1b:
        end = time.time()
        print("Frames: ", frames)
        cv2.destroyAllWindows()
        break


total_time = end - start
print("Total time:", total_time)
print("FPS:", frames/total_time)


#clean up and stop accessing camera -- important so other programs can access camera afterwards
camera.runFeatureCommand('AcquisitionStop')
# output.release()      ###### for recording video
# output_bw.release()   ###### for recording video
camera.endCapture()
camera.revokeAllFrames()
vimba.shutdown()
