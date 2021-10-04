import threading
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import queue as Queue
from timeit import default_timer as timer   #debugging
import time
import numpy as np
from PIL import ImageGrab
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
		self.sequentialAcqList[0].setRefState(False)

		# Switch to brightfield settings
		self.motor.moveFilter(1)
		self.motor.lightSwitch('white', False)
		self.motor.lightSwitch('blue', False)
		self.motor.lightSwitch('red', False)
		self.motor.lightSwitch('trans', True)
		self.partialAcqList[0].setExpTime(self.scanSettings['brightExp'])

		#print("[ScanManager/run] Start")

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

		takeFluorescence = self.scanSettings['takeFluorescence']
		frames = self.scanSettings['frames']
		step = self.scanSettings['step']
		calFreq = self.scanSettings['calFreq']
		imageSize = 9000.0/self.scanSettings['magnification'] # FLIR camera has 4.5 um/px, cropped to 2000 px
		dataPath = self.scanSettings['dataPath'] # Path for saving screenshot at end of scan
		screenshots = [] # Empty list for storing screenshots
		distX = 0.0 # Keep track of relative dist travelled in x-direction w.r.t imageSize to capture full field with FLIR camera
		distY = 0.0
		indices = np.array([]).astype(int)
		partialAcqNum = 0 # Keep track of number of FLIR camera images acquired
		motorCoords = np.empty([frames[0]*frames[1]*frames[2],3]) # Keep track of coordinates
		calFreqRead = np.empty([frames[1]*frames[2], calFreq.shape[0]]) # Keep track of actual microwave freq

		if takeFluorescence:
			print('[ScanManager] Acquiring both brightfield + fluorescence images')

		# Backlash correction before scan start:
		if step[0] > 0:
			for m in range(6):
				self.motor.moveRelative('x', -step[0])
			for m in range(6):
				self.motor.moveRelative('x', step[0])
		if step[1] > 0:
			for m in range(6):
				self.motor.moveRelative('y', -step[1])
			for m in range(6):
				self.motor.moveRelative('y', step[1])
		# Actual scan start
		for i in range(frames[2]):
			print('Frame %d of %d' %(i+1, frames[2]))
			for j in range(frames[1]):
				for k in range(frames[0]):
					#print('i,j,k = %d %d %d' % (i,j,k))
					# Check if scan cancelled
					if self.Cancel_Flag == True:
						print('[ScanManager/run] Cancel_Flag! Terminating scan...')
						# Return to start location
						if (step[0] > 0) and (k > 0):
							for n in range(k):
								self.motor.moveRelative('x', -step[0])
							#self.motor.moveAbs('x', motorCoords[0,0])
						if (step[1] > 0) and (j > 0):
							for n in range(j):
								self.motor.moveRelative('y', -step[1])
							#self.motor.moveAbs('y', motorCoords[0,1])
						if (step[2] > 0) and (i > 0):
							for n in range(i):
								self.motor.moveRelative('z', -step[2])
							#self.motor.moveAbs('z', motorCoords[0,2])
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
					# Check if 1st X step or distX > half image size (FLIR camera), if so, take new widefield image
					if np.abs(distX) >= 0.5*imageSize or k == 0:
						# Signal all devices to start new acquisition
						for dev in self.sequentialAcqList + self.partialAcqList:
							dev.continueEvent.set()
						# synchronization... wait for all the device threads to complete
						for dev in self.sequentialAcqList + self.partialAcqList:
							dev.completeEvent.wait()
							dev.completeEvent.clear()
						# Reset distX to 0 (relative) distance
						distX = 0.0
						# Check if 1st Y step or distY > half image size (FLIR camera), if so save index
						if np.abs(distY) >= 0.5*imageSize or j == 0:
							indices = np.append(indices, partialAcqNum)
						# Increment count of FLIR images acquired
						partialAcqNum += 1
						# Check if fluorescence active, if so, take (only) fluorescence image
						if takeFluorescence:
							# Switch to fluorescence settings
							self.motor.moveFilter(2)
							self.motor.lightSwitch('white', True)
							self.motor.lightSwitch('blue', True)
							self.motor.lightSwitch('red', True)
							self.motor.lightSwitch('trans', False)
							self.partialAcqList[0].setExpTime(self.scanSettings['fluorExp'])
							# Signal all devices to start new acquisition
							for dev in self.partialAcqList:
								dev.continueEvent.set()
							# synchronization... wait for all the device threads to complete
							for dev in self.partialAcqList:
								dev.completeEvent.wait()
								dev.completeEvent.clear()
							# Switch back to brightfield settings
							self.motor.moveFilter(1)
							self.motor.lightSwitch('white', False)
							self.motor.lightSwitch('blue', False)
							self.motor.lightSwitch('red', False)
							self.motor.lightSwitch('trans', True)
							self.partialAcqList[0].setExpTime(self.scanSettings['brightExp'])
					else:
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
					self.motorPosUpdateSig.emit(motorPos)
					# Move one X step forward if not end of line
					if k < frames[0]-1:
						if step[0] > 0:
							self.motor.moveRelative('x', step[0])
							distX += step[0]
					else:
						# Check if end of frame, and if so, take a screenshot
						if j==frames[1]-1 and k==frames[0]-1:
							screenshot = ImageGrab.grab(bbox=None)
							screenshots.append(screenshot)
						# take calibration data at end of line
						self.shutter.setShutterState((0, 1)) # switch to reference arm
						self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
						self.sequentialAcqList[0].setRefState(True)
						for idx, f in enumerate(calFreq):
							self.synth.setFreq(f)
							time.sleep(0.01)
							calFreqRead[i*frames[1] + j, idx] = self.synth.getFreq()
							# Signal all devices to start new acquisition
							for dev in self.sequentialAcqList:
								dev.continueEvent.set()
							# synchronization... wait for all the device threads to complete
							for dev in self.sequentialAcqList:
								dev.completeEvent.wait()
								dev.completeEvent.clear()
						# return to start position after end of line
						if step[0] > 0:
							for m in range(frames[0]+5): # frames[0]-1 + 6
								self.motor.moveRelative('x', -step[0])
							for m in range(6):
								self.motor.moveRelative('x', step[0])
						distX = 0.0 # reset relative x-distance tracker to zero
						# return to sample arm
						self.shutter.setShutterState((1, 0))
						self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
						self.sequentialAcqList[0].setRefState(False)
				if j < frames[1]-1:
					if step[1] > 0:
						self.motor.moveRelative('y', step[1])
						# reset distance tracker if just saved indices
						if np.abs(distY) >= 0.5*imageSize:
							distY = 0.0
						distY += step[1]
				else:
					# return to start position after end of frame
					if step[1] > 0:
						for n in range(frames[1]+5): # frames[1]-1 + 6
							self.motor.moveRelative('y', -step[1])
						for n in range(6):
							self.motor.moveRelative('y', step[1])
					distY = 0.0
			if i < frames[2]-1:
				if step[2] > 0:
					self.motor.moveRelative('z', step[2])
			else:
				# return to start position after end of frame
				if step[2] > 0:
					for n in range(frames[2]-1):
						self.motor.moveRelative('z', -step[2])
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
			while devProcessor.processedData.qsize() > frames[0]*frames[1]*frames[2] + calFrames*frames[1]*frames[2]:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])
		for (dev, devProcessor) in zip(self.partialAcqList, self.partialProcessingList):
			while devProcessor.processedData.qsize() > partialAcqNum:
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
		#endTime = timer()
		#print("[ScanManager] make data arrays processing time = %.3f s" % (endTime - startTime))
		# Free up memory used by dataset
		del dataset
		#startTime = timer()
		# Separate sample and reference frames
		for i in range(frames[0],0,-1):
			calPeakDist = np.delete(calPeakDist, np.s_[::i+calFrames], 0)
		#endTime = timer()
		#print("[ScanManager] delete unneeded frames processing time = %.3f s" % (endTime - startTime))
		if takeFluorescence:
			volumeScan.BrightfieldImage = CMOSImage[indices] # Only save one CMOS image every 1/2 imageSize in Y
			volumeScan.FluorescenceImage = CMOSImage[indices+1] # Only save one CMOS image every 1/2 imageSize in Y
		else:
			volumeScan.BrightfieldImage = CMOSImage[indices] # Only save one CMOS image every 1/2 imageSize in Y
			volumeScan.FluorescenceImage = np.zeros(1)
		volumeScan.MotorCoords = motorCoords
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
		# Save screenshots directly as image files
		for n, s in enumerate(screenshots):
			s.save(dataPath + 'Screenshot_Exp_%d_Scan_%d_Frame_%d.jpg' %(self.saveExpIndex, scanIdx, n))

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList + self.partialAcqList, self.sequentialProcessingList + self.partialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		print("[ScanManager/run] finished")