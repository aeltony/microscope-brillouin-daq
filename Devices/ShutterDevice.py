#import BrillouinDevice
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
from ctypes import *

# ShutterDevice does not need to run in its own thread, so we don't implement a getData() method
class ShutterDevice:

	usbObjCode = 384	# Objective shutter
	usbRefCode = 338	# Reference shutter
	usbBGCode = 339		# BG subtraction shutter

	SAMPLE_STATE = (1, 0)
	REFERENCE_STATE = (0, 1)
	CLOSED_STATE = (0, 0)
	OPEN_STATE = (1, 1)

	def __init__(self, app, state=None):
		error = c_int()
		self.dll = WinDLL('C:\\Users\\Mandelstam\\.conda\\envs\\dat_acq_py38\\PiUsb')
		self.dll.piConnectShutter.restype = c_longlong

		#connecting to shutters
		self.usbObj = c_longlong(self.dll.piConnectShutter(byref(error), ShutterDevice.usbObjCode))
		if error.value != 0:
			print('[ShutterDevice] Failed to connect to sample shutter,', ERROR_CODE[error.value])
		self.usbRef = c_longlong(self.dll.piConnectShutter(byref(error), ShutterDevice.usbRefCode))
		if error.value != 0:
			print('[ShutterDevice] Failed to connect to reference shutter,', ERROR_CODE[error.value])
		self.usbBG = c_longlong(self.dll.piConnectShutter(byref(error), ShutterDevice.usbBGCode))
		if error.value != 0:
			print('[ShutterDevice] Failed to connect to BG subtraction shutter,', ERROR_CODE[error.value])
		self.openBGshutter() # Open BG subtraction shutter
		if (state == None):
			state = ShutterDevice.SAMPLE_STATE
		self.setShutterState(state)
		self.state = state

	def shutdown(self):
		if self.usbObj != None:
			self.dll.piDisconnectShutter(self.usbObj)
		if self.usbRef != None:
			self.dll.piDisconnectShutter(self.usbRef)
		if self.usbBG != None:
			self.dll.piDisconnectShutter(self.usbBG)
		print("[ShutterDevice] Closed")
		
	# state is a tuple of (Objective, Reference)
	def setShutterState(self, state):
		error = self.dll.piSetShutterState(state[0], self.usbObj)
		if error != 0:
			print('[ShutterDevice/setShutterState] Error', ERROR_CODE[error])
		error = self.dll.piSetShutterState(state[1], self.usbRef)
		if error != 0:
			print('[ShutterDevice/setShutterState] Error', ERROR_CODE[error])
		self.state = state
		#print("[ShutterDevice] (ObjShutter, RefShutter) = (%d, %d)" % (state[0], state[1]))

	# Close shutter after fiber for BG subtraction
	def closeBGshutter(self):
		error = self.dll.piSetShutterState(0, self.usbBG)
		if error != 0:
			print('[ShutterDevice/closeBGshutter] Error', ERROR_CODE[error])

	# Open BG subtraction shutter
	def openBGshutter(self):
		error = self.dll.piSetShutterState(1, self.usbBG)
		if error != 0:
			print('[ShutterDevice/openBGshutter] Error', ERROR_CODE[error])

	def getShutterState(self):
		objState = c_int()
		refState = c_int()
		error = self.dll.piGetShutterState(byref(objState), self.usbObj)
		if error != 0:
			print('[ShutterDevice/getShutterState] Error', ERROR_CODE[error])
		error = self.dll.piGetShutterState(byref(refState), self.usbRef)
		if error != 0:
			print('[ShutterDevice/getShutterState] Error', ERROR_CODE[error])
		return (objState.value, refState.value)

ERROR_CODE = {
    0: 'PI_NO_ERROR',
    1: 'PI_DEVICE_NOT_FOUND',
    2: 'PI_OBJECT_NOT_FOUND',
	3: 'PI_CANNOT_CREATE_OBJECT',
	4: 'PI_INVALID_DEVICE_HANDLE',
	5: 'PI_READ_TIMEOUT',
	6: 'PI_READ_THREAD_ABANDONED',
	7: 'PI_READ_FAILED',
	8: 'PI_INVALID_PARAMETER',
	9: 'PI_WRITE_FAILED'
	}