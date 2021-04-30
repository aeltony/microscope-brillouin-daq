import Devices.BrillouinDevice
import time
import PySpin
import cv2
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import numpy as np

# Called "Mako" for historical reasons, this is a FLIR camera.
# This is the brightfield/epifluorescence CMOS camera.

class MakoDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(MakoDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Mako"
        self.system = None
        self.cam_list = None
        self.camera = None
        self.imageHeight = 2000 # Max 2200
        self.imageWidth = 2000 # Max 3208
        self.bin_size = 1
        self.system = PySpin.System.GetInstance()
        # Retrieve list of cameras from the system
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize()==1:
            self.camera = self.cam_list[0]
            #gentlNodemap = camera.GetTLDeviceNodeMap()
            try:
                self.camera.Init()
            except PySpin.SpinnakerException as ex:
                print(str(ex))
            #nodeMap = camera.GetNodeMap()
            print('[MakoDevice] FLIR camera initialized')
        else:
            print('[MakoDevice] FLIR camera could not be initialized')
        self.set_up()
        self.mako_lock = app.mako_lock
        self.runMode = 0    #0 is free running, 1 is scan
    
    # set up default parameters
    def set_up(self):
        try:
            self.camera.Width.SetValue(self.imageWidth)
            self.camera.Height.SetValue(self.imageHeight)
            self.camera.OffsetX.SetValue(int(0.5*(3208 - self.imageWidth)))
            self.camera.OffsetY.SetValue(int(0.5*(2200 - self.imageHeight)))
            self.camera.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
            self.camera.ExposureTime.SetValue(20000) # us
            self.camera.GainAuto.SetValue(PySpin.GainAuto_Off)
            self.camera.Gain.SetValue(0) # dB
            self.camera.AcquisitionFrameRateEnable.SetValue(True)
            self.camera.AcquisitionFrameRate.SetValue(5) # Hz
            self.camera.AcquisitionMode.SetValue(1) # 0 = Continuous, 1 = Single Frame
        except PySpin.SpinnakerException as ex:
            print(str(ex))
        print('[MakoDevice] Set-up complete')

    def __del__(self):
        return

    def shutdown(self):
        print("[MakoDevice] Closing Device")
        try:
            self.camera.DeInit()
            self.camera = None
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            print("[MakoDevice] Not closed properly")

    # getData() acquires an image from Mako
    def getData(self):
        with self.mako_lock:
            self.camera.BeginAcquisition()
            #imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
            try:
                self.image_result = self.camera.GetNextImage(1000)
            except:
                print('[MakoDevice] Empty buffer when acquiring image')
                imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
                image_arr = np.ndarray(buffer = imgData,
                                       dtype = np.uint8,
                                       shape = (self.imageHeight,self.imageWidth))
                return image_arr
            if self.image_result.IsIncomplete():
                print('[MakoDevice] Image incomplete with image status %d ...' % self.image_result.GetImageStatus())
                imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
                image_arr = np.ndarray(buffer = imgData,
                                       dtype = np.uint8,
                                       shape = (self.imageHeight,self.imageWidth))
                return image_arr
            else:
                width = self.image_result.GetWidth()
                height = self.image_result.GetHeight()
                #print('[MakoDevice] Grabbed Image with width = %d, height = %d' % (width, height))
            image_arr = np.ndarray(buffer = self.image_result.GetNDArray(),
                            dtype = np.uint8,
                            shape = (height,width))
            self.image_result.Release()
            self.camera.EndAcquisition()
        return image_arr

    def setExpTime(self, expTime):
        #print('[MakoDevice] setExpTime got called with value=', expTime)
        self.changeSetting(self.mako_lock, lambda:self.camera.ExposureTime.SetValue(expTime*1000))
        #print("[MakoDevice] Exposure time set to %.3f ms" % expTime)

    def setFrameRate(self, frameRate):
        #print('[MakoDevice] setFrameRate got called with value=', frameRate)
        self.changeSetting(self.mako_lock, lambda:self.camera.AcquisitionFrameRate.SetValue(frameRate))
        #print("[MakoDevice] Frame rate set to %.3f Hz" % frameRate)

# This class does the computation for free running mode, mostly displaying
# to the GUI
class MakoFreerun(Devices.BrillouinDevice.DeviceProcess):
    updateCMOSImageSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(MakoFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

    # data is an numpy array of type int32
    def doComputation(self, data):
        #image = np.flip(data.transpose((1,0)),1)
        image = np.flip(data, (0, 1))
        self.updateCMOSImageSig.emit(image)
        return image