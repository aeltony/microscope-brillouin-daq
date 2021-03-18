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
        self.set_up()
        self.andor_lock = app.andor_lock
        self.runMode = 0    #0 is free running, 1 is scan

        # buffer for Andor DLL image acquisition
        c_int32_p = POINTER(c_int32)
        self.imageBuffer = np.array([0 for i in range(self.cam.width*self.cam.height*2)])
        self.imageBuffer = self.imageBuffer.astype(np.int32)
        self.imageBufferPointer = self.imageBuffer.ctypes.data_as(c_int32_p)

        self.autoExp = False

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
        self.cam.SetHeight(2048)
        self.cam.SetWidth(2048)
        self.cam.SetExposureTime(.1)
        self.cam.SetCoolerMode(True)  # Continuous cooling
        self.cam.GetTemperature()
        while self.cam.temperature > 5:
            self.cam.GetTemperature()
            print("[AndorDevice] Camera cooling down, current T: %.2f C" % self.cam.temperature)
            time.sleep(1)


    def __del__(self):
        return 0

    # getData() acquires an image from Andor
    def getData(self):
        if self.autoExp:
            (im_arr, expTime) = self.getData2()
        else:
            with self.andor_lock:
                self.cam.StartAcquisition()
                self.cam.GetAcquiredData2(self.imageBufferPointer)
            expTime = self.getExposure()
            imageSize = int(self.cam.GetAcquiredDataDim())
            # return a copy of the data, since the buffer is reused for next frame
            im_arr = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        return (im_arr, expTime)


    def getData2(self):
        print("[Andor] getData2 begin")
        testExpTime = .05
        countsTarget = 15000.
        self.cam.SetExposureTime(testExpTime)
        with self.andor_lock:
            self.cam.StartAcquisition()
            self.cam.GetAcquiredData2(self.imageBufferPointer)
        imageSize = int(self.cam.GetAcquiredDataDim())
        testImage = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        maxCounts = np.amax(testImage)
        print('maxCounts =', maxCounts)
        adjustedExpTime = countsTarget*testExpTime/maxCounts
        if adjustedExpTime > 2:
            adjustedExpTime = 2
        print('adjustedExpTime =', adjustedExpTime)
        self.cam.SetExposureTime(adjustedExpTime)
        with self.andor_lock:
            self.cam.StartAcquisition()
            self.cam.GetAcquiredData2(self.imageBufferPointer)
        imageSize = self.cam.GetAcquiredDataDim()
        # return a copy of the data, since the buffer is reused for next frame
        im_arr = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        return (im_arr, adjustedExpTime)


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

    def setExposure(self, exposureTime):
        #print('[AndorDevice] setExposure got called!')
        with self.andor_lock:
            try:
                reply = self.cam.SetExposureTime(exposureTime)
                #print('reply = ', reply)
            except:
                print('[AndorDevice] Could not setExposure')
        #self.changeSetting(self.andor_lock, lambda:self.cam.SetExposureTime(exposureTime))
        print("[AndorDevice] Exposure set to %f s" % exposureTime)

    def forceSetExposure(self, exposureTime):
        #print('[AndorDevice] forceSetExposure got called!')
        try:
            reply = self.cam.SetExposureTime(exposureTime)
            #print('reply =', reply)
        except:
            print('[AndorDevice] Could not forceSetExposure')
        print("[AndorDevice] Exposure set to %f s" % exposureTime)

    #def setTemperature(self, desiredTemp):
    #    if (desiredTemp < 0 or desiredTemp > 20):
    #        print("[AndorDevice/setTemperature] Temperature out of range")
    #        return
    #    self.changeSetting(self.andor_lock, lambda:self.cam.SetTemperature(int(desiredTemp)))
    #    print("[AndorDevice] Temperature set to %d" % desiredTemp)

    def getTemperature(self):
        temp = self.getAndorSetting(self.cam.GetTemperature, 'temperature')
        #print("[AndorDevice] Temperature = %f" % temp)
        return temp

    def setAutoExp(self, autoExpStatus):
        self.autoExp = autoExpStatus
        print('autoExpStatus =', autoExpStatus)

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

        self._sampleSpectCenter = 255 # default value
        self._sampleSlineIdx = 32 # default value
        self.binHeight = 10 # number of vertical pixels to bin
        self.cropHeight = 25 # typical: 3
        self.cropWidth = 105 # typical: 50
        self.extraCrop = 35

    @property
    def sampleSpectCenter(self):
        with self.flagLock:
            return self._sampleSpectCenter

    # you can use sampleSpectCenter as if it were a class attribute, e.g.
    # pixel = self.sampleSpectCenter
    # self.sampleSpectCenter = spectrumCenter
    @sampleSpectCenter.setter
    def sampleSpectCenter(self, spectrumCenter):
        with self.flagLock:
            self._sampleSpectCenter = spectrumCenter

    @property
    def sampleSlineIdx(self):
        with self.flagLock:
            return self._sampleSlineIdx

    @sampleSlineIdx.setter
    def sampleSlineIdx(self, slineIndex):
        with self.flagLock:
            self._sampleSlineIdx = slineIndex

    # data is an numpy array of type int32
    def doComputation(self, data):
        image_array = data[0] # np.array(data, dtype = np.uint16)
        exp_time = data[1]
        #print('exp_time = ', exp_time)
        proper_image = np.reshape(image_array, (-1, 2048))   # 2048 columns
        proper_image = np.rot90(proper_image, 1, (1,0)) # Rotate by 90 deg.

        # Find spectral line
        sline_idx = self.sampleSlineIdx
        mid = self.sampleSpectCenter
        if (sline_idx < self.cropHeight):
            loc = self.cropHeight
        elif (sline_idx >= proper_image.shape[0]-self.cropHeight):
            loc = proper_image.shape[0]-self.cropHeight-1
        else:
            loc = sline_idx
        # Crop image to ROI
        cropped_image = proper_image[loc-self.cropHeight:loc+self.cropHeight, mid-self.cropWidth:mid+self.cropWidth]
        # Perform software binning
        binned_image = cropped_image.reshape(-1, self.binHeight, cropped_image.shape[-1]).sum(1)
        sline = binned_image[int(round(self.cropHeight/self.binHeight)), :]

        # Create images for GUI display
        scaled_image = cropped_image*(255.0/cropped_image.max())
        scaled_image = scaled_image.astype(int)
        scaled_8bit = np.array(scaled_image, dtype = np.uint8)
        scaled_binned = binned_image*(255.0/binned_image.max())
        scaled_binned = scaled_binned.astype(int)
        binned_8bit = np.array(scaled_binned, dtype = np.uint8)
        # Resize images to fit GUI window
        image = cv2.resize(scaled_8bit, (0,0), fx=850/(2*self.cropWidth), fy=200/(2*self.cropHeight), \
            interpolation = cv2.INTER_NEAREST)
        binned = cv2.resize(binned_8bit, (0,0), fx=420/(2*self.cropWidth), fy=105/(2*self.cropHeight/self.binHeight), \
            interpolation = cv2.INTER_NEAREST)

        # Crop sline smaller for fitting
        sline_crop = sline[self.extraCrop:-self.extraCrop]

        #### Fitting Brillouin spectrum
        interPeakDist, fittedSpect = DataFitting.fitSpectrum(np.copy(sline_crop.astype(float)),1e-4,1e-4,50)
        # emit signals for GUI to update in real time
        self.updateSampleBrillouinSeqSig.emit(interPeakDist)
        self.updateSampleSpectrum.emit((np.copy(sline_crop), np.copy(fittedSpect)))
        self.updateSampleImageSig.emit(np.copy(image))
        self.updateBinnedImageSig.emit(np.copy(binned))

        # return value is pushed into a Queue, which is collected by the ScanManager
        # for global processing (i.e. Brillouin value segmentation)
        return (proper_image, sline_crop, image, exp_time)