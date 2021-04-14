import threading
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import queue as Queue
from timeit import default_timer as timer   #debugging
import time
import numpy as np
from ExperimentData import *
import DataFitting

# Scans are sequential measurements comprised of one or many different pieces 
# of hardware. A subset of the data acqusition are taken sequentially while 
# others can be free running in their own threads. The ScanManager controls the 
# sequential data acqusitions. Processing of data from individual instruments are done in 
# their corresponding threads asynchronously from both the scan manager and the 
# data acqusition threads. 
# The scan manager also synchronizes the processed data
# from the different processing threads (and does a final processing combining
# different pieces of data?) and sends a signal to the GUI thread for live update.

class ScanManager(QtCore.QThread):
	motorPosUpdateSig = pyqtSignal(list)
	clearGUISig = pyqtSignal()

	def __init__(self, stop_event, motor, shutter, synth):
		super(ScanManager,self).__init__()
        # TODO: change to dictionary
		self.sequentialAcqList = []
		self.sequentialProcessingList = []
		self.partialAcqList = []
		self.partialProcessingList = []
		self.stop_event = stop_event

		self.motor = motor
		self.shutter = shutter
		self.synth = synth

		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method
		self.scanSettings = None
		self.Cancel_Flag = False
		self.SDcal = np.nan
		self.FSRcal = np.nan

	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		self.scanSettings = settings

	# deviceThread is a BrillouinDevice object
	# Make sure only to use thread-safe parts of BrillouinDevice, like
	# setting/clearing events or flags
	def addToSequentialList(self, deviceThread, processingThread):
		self.sequentialAcqList.append(deviceThread)
		self.sequentialProcessingList.append(processingThread)
	# PartialAcqList is for data that is only acquired during parts of the scan
	def addToPartialList(self, deviceThread, processingThread):
		self.partialAcqList.append(deviceThread)
		self.partialProcessingList.append(processingThread)

  	# In a sequential scan, the following sequence must be followed
  	#	dev.doSomethingStart()
  	#	dev.doSomethingWait()
  	#	dev.doSomethingContinue()
  	#
  	#   devThread.continueEvent.set()
    #   devThread.completeEvent.wait()
    #   devThread.completeEvent.clear()
	def run(self):
		self.setPriority(QtCore.QThread.TimeCriticalPriority)

		# make sure saving settings are ok
		if self.sessionData is None:
			print("[ScanManager] No Session provided to save data in; set ScanManager.sessionData first")
			return
		if self.saveExpIndex == -1:
			print("[ScanManager] Save parameter is empty; set ScanManager.saveParameter first")
			return

		# Switch to sample arm
		self.shutter.setShutterState((1, 0))
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
		self.sequentialAcqList[0].pauseBGsubtraction(False)

		print("[ScanManager/run] Start")

		# first turn off free running mode
		for dev in self.sequentialAcqList + self.partialAcqList:
			dev.sendPauseSignal()
		for dev in self.sequentialAcqList + self.partialAcqList:
			dev.waitForPause()
			dev.runMode = 1

		# free running mode off now; safe to force hardware settings
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])

		# Pause all processors and clear any data 
		for devProcessor in self.sequentialProcessingList + self.partialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)
			devProcessor.enqueueData = True
			while not devProcessor.processedData.empty():
				devProcessor.processedData.get()
			
		# Send signal to clear GUI plots
		self.clearGUISig.emit()

		for dev in self.sequentialAcqList + self.partialAcqList:
			dev.unpause()

		frames = self.scanSettings['frames']
		step = self.scanSettings['step']
		calFreq = self.scanSettings['calFreq']
		motorCoords = np.empty([frames[0]*frames[1]*frames[2],3]) # Keep track of coordinates
		calFreqRead = np.empty([frames[1]*frames[2], calFreq.shape[0]]) # Keep track of actual microwave freq

		for i in range(frames[2]):
			for j in range(frames[1]):
				for k in range(frames[0]):
					#print('i,j,k = %d %d %d' % (i,j,k))
					# Check if scan cancelled
					if self.Cancel_Flag == True:
						print('[ScanManager/run] Cancel_Flag! Terminating scan...')
						# Return to start location
						self.motor.moveAbs('x', motorCoords[0,0])
						self.motor.moveAbs('y', motorCoords[0,1])
						self.motor.moveAbs('z', motorCoords[0,2])
						# Stop acquiring data
						for (dev, devProcessor) in zip(self.sequentialAcqList + self.partialAcqList, self.sequentialProcessingList + self.partialProcessingList):
							devProcessor.enqueueData = False
							dev.runMode = 0
						# Wait for all processing threads to complete + empty the data queues (free up working memory)
						for devProcessor in self.sequentialProcessingList + self.partialProcessingList:
							while devProcessor.isIdle == False:
								time.sleep(0.1)
							while not devProcessor.processedData.empty():
								devProcessor.processedData.get()
						# Send signal to clear GUI plots
						self.clearGUISig.emit()
						self.maxScanPoints = 400 # Re-scale plot window for free-running mode
						# Send motor position signal to update GUI
						motorPos = self.motor.updatePosition()
						self.motorPosUpdateSig.emit(motorPos)
						return
					# Signal all devices to start new acquisition
					for dev in self.sequentialAcqList:
						dev.continueEvent.set()
					# Synchronization... wait for all the device threads to complete
					for dev in self.sequentialAcqList:
						dev.completeEvent.wait()
						dev.completeEvent.clear()
					# Send motor position signal to update GUI
					motorPos = self.motor.updatePosition()
					#print('motorPos =', motorPos)
					motorCoords[i*frames[1]*frames[0] + j*frames[0] + k] = np.array(motorPos)
					#motorCoords = np.vstack((motorCoords, np.array(motorPos)))
					self.motorPosUpdateSig.emit(motorPos)
					# Move one X step forward/backward if not end of line
					if k < frames[0]-1:
						if (i+j)%2 == 0:
							self.motor.moveRelative('x', step[0])
							#self.motor.setMotorAsync('moveRelative', 'x', [step[0]])
						else:
							self.motor.moveRelative('x', -step[0])
							#self.motor.setMotorAsync('moveRelative', 'x', [-step[0]])
					else: # take calibration data at end of line
						self.shutter.setShutterState((0, 1)) # switch to reference arm
						self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
						self.sequentialAcqList[0].pauseBGsubtraction(True)
						for idx, f in enumerate(calFreq):
							self.synth.setFreq(f)
							time.sleep(0.01)
							calFreqRead[i*frames[1] + j, idx] = self.synth.getFreq()
							# Signal all devices to start new acquisition
							for dev in self.sequentialAcqList + self.partialAcqList:
								dev.continueEvent.set()
							# synchronization... wait for all the device threads to complete
							for dev in self.sequentialAcqList + self.partialAcqList:
								dev.completeEvent.wait()
								dev.completeEvent.clear()
						# return to sample arm
						self.shutter.setShutterState((1, 0))
						self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
						self.sequentialAcqList[0].pauseBGsubtraction(False)
				if j < frames[1]-1:
					if i%2 == 0:
						self.motor.moveRelative('y', step[1])
						#self.motor.setMotorAsync('moveRelative', 'y', [step[1]])
					else:
						self.motor.moveRelative('y', -step[1])
						#self.motor.setMotorAsync('moveRelative', 'y', [-step[1]])
			if i < frames[2]-1:
				self.motor.moveRelative('z', step[2])
				#self.motor.setMotorAsync('moveRelative', 'z', [step[2]])

		# Return to start location
		self.motor.moveAbs('x', motorCoords[0,0])
		self.motor.moveAbs('y', motorCoords[0,1])
		self.motor.moveAbs('z', motorCoords[0,2])
		# Send motor position signal to update GUI
		motorPos = self.motor.updatePosition()
		self.motorPosUpdateSig.emit(motorPos)

		# Wait for all processing threads to complete
		for devProcessor in self.sequentialProcessingList + self.partialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)

		# Process Data
		#startTime = timer()
		calFrames = calFreq.shape[0]
		dataset = {'Andor': [], 'Mako': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > frames[0]*frames[1]*frames[2] + calFrames*frames[0]*frames[1]:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])
		for (dev, devProcessor) in zip(self.partialAcqList, self.partialProcessingList):
			while devProcessor.processedData.qsize() > calFrames*frames[0]*frames[1]:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])
		endTime = timer()
		#print("[ScanManager] dataset processing time = %.3f s" % (endTime - startTime))

		# Make data arrays
		#startTime = timer()
		# volumeScan.generateTestData(k)
		volumeScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))
		volumeScan.CalFreq = calFreqRead
		volumeScan.TempList = np.array(dataset['TempSensor'])
		volumeScan.AndorImage = np.array([d[0] for d in dataset['Andor']])
		CMOSImage = np.array(dataset['Mako'])
		volumeScan.SpecList = np.array([d[1] for d in dataset['Andor']])
		calPeakDist = np.array([d[2] for d in dataset['Andor']])
		endTime = timer()
		#print("[ScanManager] make data arrays processing time = %.3f s" % (endTime - startTime))
		# Free up memory used by dataset
		del dataset
		#startTime = timer()
		# Separate sample and reference frames
		for i in range(frames[0],0,-1):
			calPeakDist = np.delete(calPeakDist, np.s_[::i+calFrames], 0)
		endTime = timer()
		#print("[ScanManager] delete unneeded frames processing time = %.3f s" % (endTime - startTime))
		#startTime = timer()
		# Save one CMOS image per calibration step (the 2nd one)
		CMOSImage = CMOSImage[1::calFrames]
		endTime = timer()
		#print("[ScanManager] choose 2nd frames processing time = %.3f s" % (endTime - startTime))
		volumeScan.CMOSImage = CMOSImage
		volumeScan.MotorCoords = motorCoords
		volumeScan.Screenshot = self.scanSettings['screenshot']
		volumeScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		# Find SD / FSR of final calibration curve
		idx = frames[1]*frames[2]-1
		try:
			self.SDcal, self.FSRcal = DataFitting.fitCalCurve(np.copy(calPeakDist[idx*calFrames:(idx+1)*calFrames]), np.copy(calFreqRead[idx]), 1e-6, 1e-6)
			print('[ScanManager] Fitted SD =', self.SDcal)
			print('[ScanManager] Fitted FSR =', self.FSRcal)
		except:
			self.SDcal = np.nan
			self.FSRcal = np.nan
		# Saved fitted SD/FSR
		volumeScan.SD = self.SDcal
		volumeScan.FSR = self.FSRcal

		self.sessionData.experimentList[self.saveExpIndex].addScan(volumeScan)
		scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
		self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])])

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList + self.partialAcqList, self.sequentialProcessingList + self.partialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		print("[ScanManager/run] finished")