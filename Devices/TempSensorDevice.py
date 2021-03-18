import Devices.BrillouinDevice
import sys
import time
import traceback

from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

import numpy as np
from timeit import default_timer as timer   #debugging

from Phidget22.Devices.TemperatureSensor import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

# This is the Temperature sensor.

class TempSensorDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(TempSensorDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "TempSensor"
        self.currentTemp = -1

        self.set_up()
        print("[TempSensorDevice] Temperature sensor found")
        self.TempSensor_lock = app.TempSensor_lock
        self.runMode = 0    #0 is free running, 1 is scan

    # set up default parameters
    def set_up(self):
        # Allocate a new Phidget Channel object
        self.ch = TemperatureSensor()
        self.ch.setDeviceSerialNumber(562607) # VINT Hub serial number
        self.ch.setHubPort(0) # Port RTD Phidget is connected to
        self.ch.setIsHubPortDevice(0) # Not a networked device
        self.ch.setChannel(0) # Channel 0 = temperature sensor
        self.ch.setOnTemperatureChangeHandler(self.onTemperatureChangeHandler2)
        try:
            self.ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            print('[TempSensorDevice] Could not find RTD Phidget.')
        self.ch.setTemperatureChangeTrigger(0)
        self.ch.setDataInterval(1000)
        print("[TempSensorDevice] Temperature reading every " + str(self.ch.getDataInterval()) + " ms")
        self.ch.setRTDType(0x1)
        self.ch.setRTDWireSetup(0x2)

    def __del__(self):
        return

    def shutdown(self):
        self.ch.close()
        print("[TempSensorDevice] Closed")

    def getData(self):
        with self.TempSensor_lock:
            temperature = self.currentTemp
        return temperature

    def onTemperatureChangeHandler2(self, ch, temperature):
        self.currentTemp = temperature

# This class does the computation for free running mode, mostly displaying
# to the GUI
class TempSensorFreerun(Devices.BrillouinDevice.DeviceProcess):
    updateTempSeqSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(TempSensorFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

    def doComputation(self, data):
        temperature = data
        # print('[doComputation] Temperature = ', temperature)
        self.updateTempSeqSig.emit(temperature)
        return temperature

