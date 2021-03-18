from andor_wrap import *
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2
import time
from scipy.optimize import curve_fit

"""
***If plotting, to close window type in command prompt: taskkill /f /im python.exe
Uses andor_wrap to access and set up camera. andor_wrap is a copy of andor.py from pyandor. 
Pyandor is a wrapper for Andor cameras and is located at https://github.com/hamidohadi/pyandor.
I do not believe I made any significant changes from andor.py to andor_wrap, I think just 
small things like print statements. I copied the file because it needs to be in the same folder 
as this file, so its functions can be accessed. The location of the DLLs to access from andor_wrap
is different because the old ones were not working properly.
Software development kit for the andor camera with useful documentation is located at:
C:\Program Files\Andor SOLIS\Drivers\Software Development Kit

"""

def andor_pic():
    #initialize camera setting
    cam = Andor()
    cam.SetReadMode(4)
    cam.SetAcquisitionMode(1)
    cam.SetTriggerMode(0)
    cam.SetImage(1,4,1,cam.width,1,cam.height)
    cam.SetShutter(1,1,0,0)
    cam.SetExposureTime(.3)
    cam.SetTemperature(-75)
    cam.SetCoolerMode(1)
    cam.GetTemperature()
    print cam.temperature

    # wait till camera is cool enough 
    while cam.temperature > -74:
        cam.CoolerON()
        cam.GetTemperature()
        print cam.temperature

    cam.SetOutputAmplifier(0)


    cam.GetNumberPreAmpGains()
    print cam.noGains

    cam.GetPreAmpGain()
    print cam.preAmpGain

    #access gain at a given index from GetPreAmpGain
    cam.SetPreAmpGain(2)

    cam.SetEMAdvanced(1)
    cam.SetEMCCDGain(300)

    #manually enter whether looking at reference or not for peak fitting
    reference = False

    #hard coded SD and FSR for now
    SD = 0.2
    FSR = 16

    #list used to hold previous 100 BS calculated below
    all_BS = []

    PlasticBS = 9.6051
    WaterBS = 5.1157

    while 1:
        cam.StartAcquisition() 
        data = []                                            
        cam.GetAcquiredData(data)

        while data == []:
            continue

        print max(data)

        #converts data into an array that can actually be be used
        image_array = np.array(data, dtype = np.uint16)
        maximum = image_array.max()

        #makes copy of data that can be used for data processing that is not that is not altered
        #for visualization purposes
        graph_data = list(data)  
        graph_array = np.array(graph_data, dtype = np.uint16)
        reshaped_graph = np.reshape(graph_array, (-1, 512))
        
        
        proper_image = np.reshape(image_array, (-1, 512))

        #scales EMCCD counts on a 255 white/black scale - maxium value assigned 255
        scaled_image = proper_image*(255.0/maximum)
        scaled_image = scaled_image.astype(int)
        final = np.array(scaled_image, dtype = np.uint8)

        ####Focus on bright spots ~14 pixls in y direction and 80 pixels in x direction 
        loc = np.argmax(final)/512
        left_right = final[loc].argsort()[-10:][::-1]
        left_right.sort()
        mid = int((left_right[0]+left_right[-1])/2)

        #used to find row with max peaks for data processing 
        local_row  = final[loc][mid-40:mid+40]
        cropped = final[loc-7:loc+7, mid-40:mid+40]
        x_axis = np.arange(1,81)
        y_data = reshaped_graph[loc][mid-40:mid+40]
        if len(y_data) == 0:
                continue

        if reference:
            #four quaters of focused row, for four lorentzian fittings
            q1 = np.array(y_data[:20])
            q2 = np.array(y_data[20:40])
            q3 = np.array(y_data[40:60])
            q4 = np.array(y_data[60:])

            #constants for lorentizan fitting
            constant_1 = np.amax(q1)
            constant_2 = np.amax(q2)
            constant_3 = np.amax(q3)
            constant_4 = np.amax(q4)
            constant_5 = 100

            #x0 - x values of peaks
            x0_1 = np.argmax(q1)
            x0_2 = np.argmax(q2)+20
            x0_3 = np.argmax(q3)+40
            x0_4 = np.argmax(q4)+60

            #makes plot interactive, allows for continuous mode
            plt.ion()
            #clears previous data
            plt.clf()

            #use try/except so not finding a fit does not crash prgram 
            try:
                # attempts to find curve fitting for data - for the reference, gamma values are all 1 and it seems to work fine 
                popt, pcov = curve_fit(lorentzian_reference, x_axis, y_data, p0 = np.array([1, x0_1, constant_1, 1, x0_2, constant_2, 1, x0_3, constant_3, 1, x0_4, constant_4, constant_5]))
                plt.plot(x_axis, lorentzian_reference(x_axis, *popt), 'r-', label='fit')


                measured_SD = (2*PlasticBS - 2*WaterBS) / ((x0_4 - x0_1) + (x0_3 - x0_2))
                measured_FSR = 2*PlasticBS - measured_SD*(x0_3 - x0_2)
                print "SD", measured_SD
                print "FSR", measured_FSR
                


            except:
                pass

            plt.scatter(np.arange(1,81), y_data, s = 1)
            plt.show()
            plt.pause(0.000000000001)



        else:
            try:
                constant_1 = np.amax(y_data[:40])
                constant_2 = np.amax(y_data[40:])
                x0_1 = np.argmax(y_data[:40])
                x0_2 = np.argmax(y_data[40:])+40
                delta_peaks = x0_2-x0_1
                print x0_1

                for i in xrange(40 - x0_1):
                    half_max = constant_1/2
                    if y_data[:40][x0_1+i] <= half_max:
                        gamma_1 = i*2
                        break
                for j in xrange(40 - (x0_2 - 40)):
                    half_max = constant_2/2
                    if y_data[40:][x0_2 - 40+j] <= half_max:
                        gamma_2 = j*2
                        break

                #######For this program can only show EMCCD counts and curve fitting 
                ####### or Brillouin frequence shifts at once. If want to visualize BS graph
                ####### uncomment this block and comment out code below
                # BS = (FSR - delta_peaks*SD)/2
                # all_BS.append(BS)
                # length_bs = len(all_BS)
                # plt.ion()
                # plt.clf()
                # plt.scatter(np.arange(1, length_bs+1), np.array(all_BS))
                # plt.show()
                # plt.pause(0.000000000001)


                popt, pcov = curve_fit(lorentzian, x_axis, y_data, p0 = np.array([gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, 100]))
                plt.ion()
                plt.clf()
                plt.plot(x_axis, lorentzian(x_axis, *popt), 'r-', label='fit')
                plt.scatter(np.arange(1,81), y_data, s = 1)
                plt.show()
                plt.pause(0.000000000001)

            except:
                plt.ion()
                plt.clf()
                plt.scatter(np.arange(1,81), y_data, s = 1)
                plt.show()
                plt.pause(0.000000000001)

            

        (h, w)= cropped.shape[:2]
        if w == 0 or h == 0:
            continue

        cv2.namedWindow('Data', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Data', 1024, 1024 )
        cv2.imshow('Data', cropped)

        #Useless if showing graphs, otherwise, pres ESC key to exit 
        k = cv2.waitKey(1)
        if k == 0x1b:
            cv2.destroyAllWindows()
            break


    cam.ShutDown() 

# lorentzian function used for fitting 2 peaks
def lorentzian(x, gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, constant_3):
    numerator_1 = 0.5*gamma_1*constant_1
    denominator_1 = math.pi * (x - x0_1) **2 + (0.5 * gamma_1)**2
    numerator_2 = 0.5*gamma_2*constant_2
    denominator_2 = math.pi * (x - x0_2) **2 + (0.5 * gamma_2)**2
    y = (numerator_1/denominator_1) + (numerator_2/denominator_2) + constant_3
    return y
#lorentzian function used for fitting 4 peaks
def lorentzian_reference(x, gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, gamma_3, x0_3, constant_3, gamma_4, x0_4, constant_4, constant_5):
    numerator_1 = 0.5*gamma_1*constant_1
    denominator_1 = math.pi * (x - x0_1) **2 + (0.5 * gamma_1)**2
    numerator_2 = 0.5*gamma_2*constant_2
    denominator_2 = math.pi * (x - x0_2) **2 + (0.5 * gamma_2)**2
    numerator_3 = 0.5*gamma_3*constant_3
    denominator_3 = math.pi * (x - x0_3) **2 + (0.5 * gamma_3)**2
    numerator_4 = 0.5*gamma_4*constant_4
    denominator_4 = math.pi * (x - x0_4) **2 + (0.5 * gamma_4)**2
    y = (numerator_1/denominator_1) + (numerator_2/denominator_2) + (numerator_3/denominator_3) + (numerator_4/denominator_4) + constant_5
    return y

andor_pic()