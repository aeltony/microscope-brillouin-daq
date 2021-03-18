import BrillouinDevice
from time import sleep

from PyQt4 import QtGui,QtCore
from PyQt4.QtCore import pyqtSignal


# This is one of the main devices. It simply acquires a singel set of data 
# from the Andor EMCCD when the condition AndorDevice.continueEvent() is 
# set from a managing class. 

class TestDevice(BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, delayTime, deviceIndex):
        super(TestDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Test1"
        self.delayTime = delayTime
        self.deviceIndex = deviceIndex

    def getData(self):
        # print "Andor get data %d begin" % self.counter
        # startTime = default_timer()  
        # with self.andor_lock:
        #     self.cam.StartAcquisition() 
        #     self.cam.GetAcquiredData2(self.imageBufferPointer)
        # imageSize = self.cam.GetAcquiredDataDim()
        # endTime = default_timer()  
        # print "Andor time = %f" % (endTime - startTime)
        #time.sleep(1)
        #print "[AndorDevice] frame acquired"
        #print type(self.imageBuffer)
        # print "Andor count = %d" % self.counter

        sleep(self.delayTime) #sleep 10ms

        return self.deviceIndex

