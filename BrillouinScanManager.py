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
# sequential data acqusitions and synchronizes with the free running ones using
# ??? (time tags?). Processing of data from individual instruments are done in 
# their corresponding threads asynchronously from both the scan manager and the 
# data acqusition threads. 
# The scan manager also synchronizes the processed data
# from the different processing threads (and does a final processing combining
# different pieces of data?) and sends a signal to the GUI thread for live update.

class ScanManager(QtCore.QThread):
	motorPosUpdateSig = pyqtSignal(list)
	clearGUISig = pyqtSignal()

	#TODO: add a pause event
	def __init__(self, stop_event, motor, shutter, synth):
		super(ScanManager,self).__init__()
        # TODO: change to dictionary
		self.sequentialAcqList = []
		self.sequentialProcessingList = []
		self.freerunningList = []
		self.stop_event = stop_event

		self.motor = motor
		self.shutter = shutter
		self.synth = synth

		self.processor = None

		self.saveScan = True
		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method

		self.scanSettings = None
		self.Cancel_Flag = False


	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		self.scanSettings = settings

	# deviceThread is a BrillouinDevice object
	# Make sure only to use thread-safe parts of BrillouinDevice, like
	# setting/clearing events or flags
	def addToSequentialList(self, deviceThread, processingThread):
		self.sequentialAcqList.append(deviceThread)
		self.sequentialProcessingList.append(processingThread)

	def addToFreerunningList(self, deviceThread):
		self.freerunningList.append(deviceThread)

	# processor is a method that takes a list of Brillouin shifts and other arguments as needed
	def addDataProcessor(self, processor):
		self.processor = processor

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
		if self.saveScan:
			if self.sessionData is None:
				print("No Session provided to save data in; set ScanManager.sessionData first")
				return
			if self.saveExpIndex == -1:
				print("Save parameter is empty; set ScanManager.saveParameter first")
				return

		# Switch to sample arm
		self.shutter.setShutterState((1, 0))
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
		self.sequentialAcqList[0].pauseBGsubtraction(False)

		print("[ScanManager/run] Start")

		# first turn off free running mode
		for dev in self.sequentialAcqList:
			dev.sendPauseSignal()
		for dev in self.sequentialAcqList:
			dev.waitForPause()
			dev.runMode = 1

		# free running mode off now; safe to force hardware settings
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])

		# Pause all processors and clear any data 
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)
			devProcessor.enqueueData = True
			while not devProcessor.processedData.empty():
				devProcessor.processedData.get()
			
		# Send signal to clear GUI plots
		self.clearGUISig.emit()

		for dev in self.sequentialAcqList:
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
						for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
							devProcessor.enqueueData = False
							dev.runMode = 0
						# Send signal to clear GUI plots
						self.clearGUISig.emit()
						self.maxScanPoints = 400 # Re-scale plot window for free-running mode
						# Send motor position signal to update GUI
						motorPos = self.motor.updatePosition()
						self.motorPosUpdateSig.emit(motorPos)
						self.Cancel_Flag = False
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
							for dev in self.sequentialAcqList:
								dev.continueEvent.set()
							# synchronization... wait for all the device threads to complete
							for dev in self.sequentialAcqList:
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
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)

		print('calFreqRead =', calFreqRead)
		# Process Data
		calFrames = calFreq.shape[0]
		#BS = np.random.random()*(self.colormapHigh - self.colormapLow) + self.colormapLow
		dataset = {'Andor': [], 'Mako': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > frames[0]*frames[1]*frames[2] + calFrames*frames[0]*frames[1]:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])

		# Create data arrays
		RawTempList = np.array(dataset['TempSensor'])
		CalTempList = np.copy(RawTempList)
		imageList = [d[0] for d in dataset['Mako']]
		CMOSImage = np.array(imageList)
		specImageList = [d[0] for d in dataset['Andor']]
		AndorImage = np.array(specImageList)
		CalImage = np.copy(AndorImage)
		maxRowList = [d[1] for d in dataset['Andor']]
		RawSpecList = np.array(maxRowList)
		CalSpecList = np.copy(RawSpecList)
		dispImageList = [d[2] for d in dataset['Andor']]
		AndorDisplay = np.array(dispImageList)
		laserPos = np.array([np.float(self.scanSettings['laserX']),np.float(self.scanSettings['laserY'])])

		# Separate sample and reference frames
		for i in range(calFrames,0,-1):
			RawTempList = np.delete(RawTempList, np.s_[frames[0]::frames[0]+i], 0)
			AndorImage = np.delete(AndorImage, np.s_[frames[0]::frames[0]+i], 0)
			RawSpecList = np.delete(RawSpecList, np.s_[frames[0]::frames[0]+i], 0)
			CMOSImage = np.delete(CMOSImage, np.s_[frames[0]::frames[0]+i], 0)
		for i in range(frames[0],0,-1):
			CalTempList = np.delete(CalTempList, np.s_[::i+calFrames], 0)
			CalImage = np.delete(CalImage, np.s_[::i+calFrames], 0)
			CalSpecList = np.delete(CalSpecList, np.s_[::i+calFrames], 0)

		# Save data
		# volumeScan.generateTestData(k)
		volumeScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))
		volumeScan.CalFreq = calFreqRead
		volumeScan.RawTempList = RawTempList
		volumeScan.CalTempList = CalTempList
		volumeScan.AndorImage = AndorImage
		volumeScan.CalImage = CalImage
		volumeScan.CMOSImage = CMOSImage
		volumeScan.RawSpecList = RawSpecList
		volumeScan.CalSpecList = CalSpecList
		volumeScan.AndorDisplay = AndorDisplay
		volumeScan.LaserPos = laserPos
		volumeScan.MotorCoords = motorCoords
		volumeScan.Screenshot = self.scanSettings['screenshot']
		volumeScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		#### Fitting Brillouin spectra
		startTime = timer()
		freqList = np.zeros(RawSpecList.shape[0])
		signal = np.zeros(RawSpecList.shape[0])
		fittedSpect = np.empty(RawSpecList.shape)
		# Find SD / FSR for every (y, z) coordinate
		SDcal = np.empty([frames[1]*frames[2]])
		FSRcal = np.empty([frames[1]*frames[2]])
		for i in range(frames[1]*frames[2]):
			pxDist = np.empty(calFreq.shape)
			for j in range(calFrames):
				interPeakDist, fittedCalSpect = DataFitting.fitSpectrum(np.copy(CalSpecList[i*calFrames+j]),1e-6,1e-6)
				if len(interPeakDist)>1:
					pxDist[j] = interPeakDist[1]
				else:
					print("[ScanManager/run] Calibration #%d failed." %j)
					pxDist[j] = np.nan
			try:
				SDcal[i], FSRcal[i] = DataFitting.fitCalCurve(np.copy(pxDist), np.copy(calFreqRead[i]), 1e-6, 1e-6)
				print('Fitted SD =', SDcal[i])
				print('Fitted FSR =', FSRcal[i])
			except:
				SDcal[i] = np.nan
				FSRcal[i] = np.nan

			for j in range(frames[0]):
				sline = np.copy(RawSpecList[i*frames[0]+j])
				sline = np.transpose(sline)
				interPeakDist, fittedSpect[i*frames[0]+j] = DataFitting.fitSpectrum(sline,1e-6,1e-6)
				if len(interPeakDist)==2:
					freqList[i*frames[0]+j] = 0.5*(FSRcal[i] - SDcal[i]*interPeakDist[1])
					signal[i*frames[0]+j] = interPeakDist[0]
				else:
					freqList[i*frames[0]+j] = np.nan
					signal[i*frames[0]+j] = np.nan
		# Saved fitted data
		volumeScan.SD = SDcal
		volumeScan.FSR = FSRcal
		volumeScan.BSList = freqList
		volumeScan.FitSpecList = fittedSpect

		endTime = timer()
		print("[ScanManager] Fitting time = %.3f s" % (endTime - startTime))
		print('Brillouin frequency shift list:', freqList)

		self.sessionData.experimentList[self.saveExpIndex].addScan(volumeScan)
		scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
		self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])])

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		print("[ScanManager/run] finished")