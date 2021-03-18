import threading
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

from time import sleep
from timeit import default_timer as default_timer   #debugging
import queue as Queue

# Device is designed to be threadsafe, i.e. can be used from both the GUI thread and also 
# passed into the Scan thread

class Device(QtCore.QThread):

    freerunningMaxQueue = 2

    def __init__(self, stop_event, runMode=0):
        super(Device,self).__init__()
        self.deviceName = None
        self.stop_event = stop_event
        # self.pauseEvent = threading.Event()
        self.runEvent = threading.Event()
        self.continueEvent = threading.Event()
        self.completeEvent = threading.Event()
        self.isPaused = threading.Event()
        self.dataQueue = Queue.PriorityQueue()
        self.flagLock = threading.Lock()
        self._enqueueData = True

        self.queueMax = Device.freerunningMaxQueue

        # setting or reading flags require the use of self.flagLock
        self._runMode = runMode    # 0 is freerunning, no waiting for event, does not set completeEvent flag after
                                # 1 is data acq only when continueEvent is set, sets completeEvent flag after each data acq

    # use property to set flags in order to force flagLock to be used
    @property
    def runMode(self):
        with self.flagLock:
            return self._runMode

    @property
    def enqueueData(self):
        with self.flagLock:
            return self._enqueueData

    @enqueueData.setter
    def enqueueData(self, flag):
        with self.flagLock:
            self._enqueueData = flag

    @runMode.setter
    def runMode(self, value):
        with self.flagLock:
            self._runMode = value
            if value == 1:
                self.queueMax = 1000000
            else:
                self.queueMax = Device.freerunningMaxQueue

    def pause(self):
        self.runEvent.clear()
        self.isPaused.wait()

    # this is useful for asynchronous signaling
    def sendPauseSignal(self):
        self.runEvent.clear()

    def waitForPause(self):
        self.isPaused.wait()

    def unpause(self):
        self.runEvent.set()        

    # reimplement this in derived classes
    def getData(self):
        return 0

    def run(self):
        self.counter = 0
        self.runEvent.set()

        while not self.stop_event.is_set():

            currentMode = self.runMode # make sure the mode doesnt change during an acquisition

            if not self.runEvent.is_set():
                self.isPaused.set()
                self.runEvent.wait()
                self.isPaused.clear()
                currentMode = self.runMode

            # if (currentMode == 1 and self.deviceName == 'Andor'):
            #     startTime = default_timer()  

            # Next chunk of code manages thread synchronization
            continueLoop = False
            if (currentMode == 1): # Note: this is in fact threadsafe
                while True:
                    ret = self.continueEvent.wait(timeout = 0.5) 
                    if (ret == False and self.runMode == 0):
                        # timeout. check if runMode has changed so that continueEvent no longer needs to be set
                        self.continueEvent.clear()
                        continueLoop = True
                        break
                    elif ret == False and self.runMode == 1:
                        continue
                    else:
                        self.continueEvent.clear()
                        # print(self.deviceName + " continue acq")
                        break
            if continueLoop:
                continue

            #if self.deviceName == 'Mako':
                #print('Mako dataQueue size = ', self.dataQueue.qsize())

            while self.dataQueue.qsize() >= self.queueMax:
                sleep(0.02)

            self.counter += 1

            data = self.getData()
            if (self.enqueueData):
                self.dataQueue.put((self.counter, data))

            if (currentMode == 1):
                self.completeEvent.set() 

            # if (currentMode == 1 and self.deviceName == 'Andor'):
            #     endTime = default_timer()  
            #     print("Andor entire time = %f" % (endTime-startTime))

        print("[Device]" + self.deviceName + " thread stopped")

    def test(self):
        print('[BrillouinDevice] Test')

    def changeSetting(self, lock, functionHandle):
        if not self.isRunning():
            with lock:
                functionHandle()
        else:
            # pause device acquisition first
            self.pause()
            with lock:
                functionHandle()
            self.unpause()

    def getSetting(self, lock, functionHandle):
        if not self.isRunning():
            with lock:
                result = functionHandle()
        else:
            # pause device acquisition first
            self.pause()
            with lock:
                result = functionHandle()
            self.unpause()
        return result


# TODO: throw exception/warning when too many elements are in queue
# TODO: make threadsafe (locks for setting flags)
class DeviceProcess(QtCore.QThread):
    finishedProcessingSig = pyqtSignal()     #All processing threads has a finished signal. Derived classes can add more if desired

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(DeviceProcess, self).__init__()
        self.stopProcessingEvent = stopProcessingEvent
        self.processedData = Queue.PriorityQueue()
        self.device = device

        self.flagLock = threading.Lock()

        self._enqueueData = False    
        self._sendSignalWhenFinish = True
        self._isIdle = True
        self._clearDataQueue = False

        self.finishedTrigger = None
        self.connectFinishedTrigger(finishedTrigger)

    @property
    def enqueueData(self):
        with self.flagLock:
            return self._enqueueData

    @enqueueData.setter
    def enqueueData(self, flag):
        with self.flagLock:
            self._enqueueData = flag

    @property
    def sendSignalWhenFinish(self):
        with self.flagLock:
            return self._sendSignalWhenFinish

    @sendSignalWhenFinish.setter
    def sendSignalWhenFinish(self, flag):
        with self.flagLock:
            self._sendSignalWhenFinish = flag

    @property
    def isIdle(self):
        with self.flagLock:
            return self._isIdle

    @isIdle.setter
    def isIdle(self, flag):
        with self.flagLock:
            self._isIdle = flag

    def clearDataQueue(self):
        with self.flagLock:
            self._clearDataQueue = True
    
    
    # derived class implements their own computation routine
    def doComputation(self, data):
        return 0

    def connectFinishedTrigger(self, finishedTrigger):
        # First disconnect existing slot
        if (self.finishedTrigger is not None):
            self.finishedProcessingSig.disconnect(self.finishedTrigger)

        if finishedTrigger is not None:
            self.finishedProcessingSig.connect(finishedTrigger)
        self.finishedTrigger = finishedTrigger

    def processData(self):
        try:
            x = self.device.dataQueue.get(block=True, timeout=0.1)  
            self.isIdle = False
            y = self.doComputation(x[1])
            if (self.enqueueData):
                self.processedData.put((x[0], y))
            if (self.sendSignalWhenFinish):
                self.finishedProcessingSig.emit()
            with self.flagLock:
                if self._clearDataQueue:
                    if not self.device.dataQueue.empty():  
                        q = self.device.dataQueue
                        q.mutex.acquire()
                        q.queue.clear()
                        q.all_tasks_done.notify_all()
                        q.unfinished_tasks = 0
                        q.mutex.release()
                    self._clearDataQueue = False

        except Queue.Empty:
            # print("[DeviceProcess/processData] queue empty")
            self.isIdle = True
            pass

    def run(self):
        while not self.stopProcessingEvent.is_set():
            self.processData()

    def printData(self):
        t = sorted([x[1] for x in list(self.processedData.queue)])
        print(t)


########################################################################
#                   Example derived class                              #
########################################################################
# class Device1(Device):
#     def __init__(self, stop_event):
#         super(Device1, self).__init__(stop_event)

#     def __del__(self):
#         return 0

#     def getData(self):
#         sleep(1)
#         print("device1 generating data")
#         return 4

# class Device1Process(DeviceProcess):
#     def __init__(self, device, stopProcessingEvent):
#         super(Device1Process, self).__init__(device, stopProcessingEvent)

#     def doComputation(self, data):
#         return data*data