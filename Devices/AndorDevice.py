import Devices.BrillouinDevice
import time
import numpy as np
import DataFitting
from Devices.Andor_DLL_wrap.andor_wrap import *
import cv2
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

# This is one of the main devices. It simply acquires a single set of data 
# from the Andor device when the condition AndorDevice.continueEvent() is 
# set from a managing class. 

class AndorDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(AndorDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Andor"
        self.cam = Andor()
        self.cam.SetVerbose(False)
        self.cam.Initialize()

        self.autoExp = False
        self.bgSubtraction = False
        self.refState = False
        self.triggerBG = False
        self.sampleExp = 0.1
        self.refExp = 0.1
        self.AOITop = 1111
        self.AOILeft = 938
        self.set_up()
        self.andor_lock = app.andor_lock
        self.runMode = 0    #0 is free running, 1 is scan

        # buffer for Andor DLL image acquisition
        c_int32_p = POINTER(c_int32)
        self.imageBuffer = np.zeros(self.cam.width*self.cam.height) #np.array([0 for i in range(self.cam.width*self.cam.height*2)])
        self.imageBuffer = self.imageBuffer.astype(np.int32)
        self.imageBufferPointer = self.imageBuffer.ctypes.data_as(c_int32_p)

    # set up default parameters
    def set_up(self):
        self.cam.SetCycleMode(u'Fixed')
        self.cam.SetTriggerMode(u'Internal')
        self.cam.SetNumberAccumulations(1)
        self.cam.SetFrameCount(1)
        self.cam.SetShutter(u'Auto')
        self.cam.SetReadoutRate(u'100 MHz')
        self.cam.SetPreAmpGain(u'16-bit (low noise & high well capacity)')
        self.cam.SetPixelEncoding(u'Mono32')
        self.cam.SetHBin(1)
        self.cam.SetVBin(1)
        self.cam.SetWidth(50)
        self.cam.SetAOILeft(self.AOILeft)
        self.cam.SetHeight(210)
        self.cam.SetAOITop(self.AOITop)
        if self.refState:
            self.cam.SetExposureTime(self.refExp)
        else:
            self.cam.SetExposureTime(self.sampleExp)
        self.cam.SetCoolerMode(True)  # Continuous cooling
        self.cam.GetTemperature()
        while self.cam.temperature > 5:
            self.cam.GetTemperature()
            print("[AndorDevice] Camera cooling down, current T: %.2f C" % self.cam.temperature)
            time.sleep(1)
        self.cam.CreateBuffer()

    def __del__(self):
        return 0

    def startBGsubtraction(self):
        self.triggerBG = True

    # Check if BG subtraction is complete
    def checkBGsubtraction(self):
        return self.triggerBG

    def stopBGsubtraction(self):
        self.bgSubtraction = False

    def setRefState(self, refStatus):
        self.refState = refStatus

    # getData() acquires an image from Andor
    def getData(self):
        if self.triggerBG:
            self.bgImage = self.getBG()
            self.triggerBG = False
            self.bgSubtraction = True
        if self.autoExp:
            im_arr = self.getData2()
        else:
            with self.andor_lock:
                try:
                    errorStr = self.cam.StartAcquisition()
                    if errorStr != 'AT_SUCCESS':
                        if errorStr == 'Restarted camera':
                            # Need to reset parameters
                            self.set_up()
                            # Try again after restart
                            print('[AndorDevice] Acquisition failed, trying again...')
                            errorStr = self.cam.StartAcquisition()
                            if errorStr != 'AT_SUCCESS':
                                print('[AndorDevice] Acquisition failed twice!')
                                im_arr = np.ones(self.cam.width*self.cam.height)
                                return im_arr
                        else:
                            im_arr = np.ones(self.cam.width*self.cam.height)
                            return im_arr
                except:
                    print('[AndorDevice] Acquisition failed!')
                    im_arr = np.ones(self.cam.width*self.cam.height)
                    return im_arr
                try:
                    errorStr = self.cam.GetAcquiredData2(self.imageBufferPointer)
                    if errorStr != 'AT_SUCCESS':
                        print('[AndorDevice] Could not get acquired data')
                        im_arr = np.ones(self.cam.width*self.cam.height)
                        return im_arr
                except:
                    print('[AndorDevice] Could not get acquired data')
                    im_arr = np.ones(self.cam.width*self.cam.height)
                    return im_arr
            #expTime = self.getExposure()
            imageSize = int(self.cam.GetAcquiredDataDim())
            # return a copy of the data, since the buffer is reused for next frame
            im_arr = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
            if self.bgSubtraction and not self.refState:
                im_arr = im_arr - self.bgImage
        return im_arr

    def getBG(self):
        #print("[Andor] getBG begin")
        with self.andor_lock:
            self.cam.StartAcquisition()
            self.cam.GetAcquiredData2(self.imageBufferPointer)
        imageSize = int(self.cam.GetAcquiredDataDim())
        bgImgArr = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        for k in np.arange(4):
            with self.andor_lock:
                self.cam.StartAcquisition()
                self.cam.GetAcquiredData2(self.imageBufferPointer)
            imageSize = int(self.cam.GetAcquiredDataDim())
            bgImgArr = bgImgArr + np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        bgImage = bgImgArr/5
        return bgImage

    #def getData2(self):
    #    print("[Andor] getData2 begin")
    #    testExpTime = .05
    #    countsTarget = 15000.
    #    self.cam.SetExposureTime(testExpTime)
    #    with self.andor_lock:
    #        self.cam.StartAcquisition()
    #        self.cam.GetAcquiredData2(self.imageBufferPointer)
    #    imageSize = int(self.cam.GetAcquiredDataDim())
    #    testImage = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
    #    maxCounts = np.amax(testImage)
    #    print('maxCounts =', maxCounts)
    #    adjustedExpTime = countsTarget*testExpTime/maxCounts
    #    if adjustedExpTime > 2:
    #        adjustedExpTime = 2
    #    print('adjustedExpTime =', adjustedExpTime)
    #    self.cam.SetExposureTime(adjustedExpTime)
    #    with self.andor_lock:
    #        self.cam.StartAcquisition()
    #        self.cam.GetAcquiredData2(self.imageBufferPointer)
    #    imageSize = self.cam.GetAcquiredDataDim()
    #    # return a copy of the data, since the buffer is reused for next frame
    #    im_arr = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
    #    return im_arr

    def getAndorSetting(self, functionHandle, attribute):
        result = 0
        if not self.isRunning():
            with self.andor_lock:
                functionHandle()
                result = self.cam.__dict__[attribute]
        else:
            # pause device acquisition first
            self.pause()
            with self.andor_lock:
                functionHandle()
                result = self.cam.__dict__[attribute]
            self.unpause()
        return result

    def getExposure(self):
        with self.andor_lock:
            return self.cam.exposure

    def setLeftPx(self, pixel):
        # pause device acquisition first
        self.pause()
        with self.andor_lock:
            try:
                reply = self.cam.SetAOILeft(pixel)
            except:
                print('[AndorDevice] Could not setLeftPx')
            self.AOILeft = pixel
            print("[AndorDevice] AOILeft set to %d px" % pixel)
        self.unpause()

    def setTopPx(self, pixel):
        # pause device acquisition first
        self.pause()
        with self.andor_lock:
            try:
                reply = self.cam.SetAOITop(pixel)
            except:
                print('[AndorDevice] Could not setTopPx')
            self.AOITop = pixel
            print("[AndorDevice] AOITop set to %d px" % pixel)
        self.unpause()

    def setExposure(self, exposureTime):
        #print('[AndorDevice] setExposure got called!')
        with self.andor_lock:
            try:
                reply = self.cam.SetExposureTime(exposureTime)
            except:
                print('[AndorDevice] Could not setExposure')
            if self.refState:
                self.refExp = exposureTime
            else:
                self.sampleExp = exposureTime
        #self.changeSetting(self.andor_lock, lambda:self.cam.SetExposureTime(exposureTime))
        #print("[AndorDevice] Exposure set to %f s" % exposureTime)

    def forceSetExposure(self, exposureTime):
        #print('[AndorDevice] forceSetExposure got called!')
        try:
            reply = self.cam.SetExposureTime(exposureTime)
            #print('reply =', reply)
        except:
            print('[AndorDevice] Could not forceSetExposure')
        if self.refState:
            self.refExp = exposureTime
        else:
            self.sampleExp = exposureTime
        #print("[AndorDevice] Exposure set to %f s" % exposureTime)

    def getTemperature(self):
        temp = self.getAndorSetting(self.cam.GetTemperature, 'temperature')
        #print("[AndorDevice] Temperature = %f" % temp)
        return temp

    #def setAutoExp(self, autoExpStatus):
    #    self.autoExp = autoExpStatus
    #    print('autoExpStatus =', autoExpStatus)

# This class does the computation for free running mode, mostly displaying
# to the GUI
# It has a handle to the Andor device which has the data queue containing
# raw frames from the camera
class AndorProcessFreerun(Devices.BrillouinDevice.DeviceProcess):
    updateSampleBrillouinSeqSig = pyqtSignal('PyQt_PyObject')
    updateSampleSpectrum = pyqtSignal('PyQt_PyObject')
    updateSampleImageSig = pyqtSignal('PyQt_PyObject')
    updateBinnedImageSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(AndorProcessFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)
        self.binHeight = 10 # number of vertical pixels to bin, typ. 10

    # data is an numpy array of type int32
    def doComputation(self, data):
        proper_image = np.reshape(data, (-1, 50))   # 50 columns
        proper_image = np.rot90(proper_image, 1, (1,0)) # Rotate by 90 deg.
        # Perform software binning
        binned_image = proper_image.reshape(-1, self.binHeight, proper_image.shape[-1]).sum(1)
        sline = binned_image[int(np.floor(0.5*proper_image.shape[0]/self.binHeight)), :]

        # Create images for GUI display
        positive_image = np.clip(proper_image, 0, 1e12)
        scaled_image = positive_image*(255.0/positive_image.max())
        scaled_image = scaled_image.astype(int)
        scaled_8bit = np.array(scaled_image, dtype = np.uint8)
        positive_binned = np.clip(binned_image, 0, 1e12)
        scaled_binned = positive_binned*(255.0/positive_binned.max())
        scaled_binned = scaled_binned.astype(int)
        binned_8bit = np.array(scaled_binned, dtype = np.uint8)
        # Resize images to fit GUI window
        image = cv2.resize(scaled_8bit, (0,0), fx=860/scaled_8bit.shape[1], fy=200/scaled_8bit.shape[0], \
            interpolation = cv2.INTER_NEAREST)
        binned = cv2.resize(binned_8bit, (0,0), fx=440/binned_8bit.shape[1], fy=100/binned_8bit.shape[0], \
            interpolation = cv2.INTER_NEAREST)

        #### Fitting Brillouin spectrum
        interPeakDist, fittedSpect = DataFitting.fitSpectrum(np.copy(sline.astype(float)),1e-4,1e-4,50)
        # emit signals for GUI to update in real time
        self.updateSampleBrillouinSeqSig.emit(interPeakDist)
        self.updateSampleSpectrum.emit((np.copy(sline), np.copy(fittedSpect)))
        self.updateSampleImageSig.emit(np.copy(image))
        self.updateBinnedImageSig.emit(np.copy(binned))

        # return value is pushed into a Queue, which is collected by the ScanManager
        # for global processing (i.e. Brillouin value segmentation)
        return (proper_image, sline, interPeakDist)