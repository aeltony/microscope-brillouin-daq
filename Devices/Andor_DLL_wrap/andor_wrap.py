#   Python wrapper for Andor cameras using SDK3
#   Amira Eltony, 2020
#   Based on pyAndor for SDK2 by Hamid Ohadi (2009)

import platform
from ctypes import *
from PIL import Image
import sys
import numpy as np
import time
from timeit import default_timer as default_timer   #debugging


"""Andor class which is meant to provide the Python version of the same
   functions that are defined in the Andor's SDK. Since Python does not
   have pass by reference for immutable variables, some of these variables
   are actually stored in the class instance. For example the temperature,
   gain, gainRange, status etc. are stored in the class. """

class Andor:
    def __init__(self):
        self.verbosity   = False
        # Load library for Windows
        self.dll = WinDLL('C:\\Users\\Mandelstam\\.conda\\envs\\dat_acq_py38\\atcore')
        self.utildll = WinDLL('C:\\Users\\Mandelstam\\.conda\\envs\\dat_acq_py38\\atutility')
        error = self.dll.AT_InitialiseLibrary('')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        error = self.utildll.AT_InitialiseUtilityLibrary('')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.handle      = c_longlong()
        self.temperature = None
        self.set_T       = None
        self.gain        = None
        self.gainRange   = None
        # self.status      = ERROR_CODE[error]
        self.channel     = None
        self.outamp      = None
        self.hsspeed     = None
        self.vsspeed     = None
        self.serial      = None
        self.exposure    = None
        self.accumulate  = None
        self.scans       = 1
        self.hbin        = 1
        self.vbin        = 1
        self.height      = 2048
        self.width       = 2048
        self.left        = 0
        self.top         = 0
        self.cooler      = None
        
    def __del__(self):
        error = self.dll.AT_FinaliseLibrary('')
        error = self.utildll.AT_FinaliseUtilityLibrary('')
    
    def verbose(self, error, function=''):
        if self.verbosity is True:
            print("[%s]: %s" %(function, error))

    def SetVerbose(self, state=True):
        self.verbosity = state

    def Restart(self):
        # Stop acquisition
        print('[AndorDevice/Restart] Stopping acquisition...')
        try:
            error = self.dll.AT_Command(self.handle, 'AcquisitionStop')
            self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
            if error != 0:
                print('[AndorDevice/Restart] Stop acq error: ' + ERROR_CODE[error])
                return error
        except:
            print('[AndorDevice/Restart] Failed to send AcquisitionStop')
        # Flush buffer
        print('[AndorDevice/Restart] Flushing buffer...')
        try:
            error = self.dll.AT_Flush(self.handle)
            self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
            if error != 0:
                print('[AndorDevice/Restart] AT_Flush error:' + ERROR_CODE[error])
                return error
        except:
            print('[AndorDevice/Restart] Failed to send AT_Flush')
        # Close camera
        print('[AndorDevice/Restart] Closing...')
        try:
            error = self.dll.AT_Close(self.handle)
            self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
            if error != 0:
                print('[AndorDevice/Restart] AT_Close error:' + ERROR_CODE[error])
                return error
        except:
            print('[AndorDevice/Restart] Failed to send AT_Close')
        # Re-open camera
        print('[AndorDevice/Restart] Re-opening...')
        try:
            error = self.dll.AT_Open(0, byref(self.handle))
            self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
            if error != 0:
                print('[AndorDevice/Restart] AT_Open error:' + ERROR_CODE[error])
                return error
        except:
            print('[AndorDevice/Restart] Failed to send AT_Open')
        self.CreateBuffer()
        return error

    def AbortAcquisition(self):
        error = self.dll.AT_Command(self.handle, 'AcquisitionStop')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def Initialize(self):
        #print('Initializing Zyla')
        error = self.dll.AT_Open(0, byref(self.handle))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        #print('self.handle =', self.handle)
        self.GetCameraSerialNumber()
        print("[AndorDevice] Zyla sCMOS camera found: ", self.serial)
        return ERROR_CODE[error]

    def CreateBuffer(self):
        # Create buffer for Andor DLL image acquisition
        self.im_size = self.GetImageSize()
        self.buffer_size = self.im_size.value
        c_int32_p = POINTER(c_int32)
        imageBuffer = np.array([0 for i in range(self.buffer_size*2)])
        imageBuffer = imageBuffer.astype(np.int32)
        self.imageBufferPointer = imageBuffer.ctypes.data_as(c_int32_p)
        
    def ShutDown(self):
        error1 = self.dll.AT_Flush(self.handle)
        if error1 != 0:
            print('[AndorDevice] AT_Flush error: ' + ERROR_CODE[error1])
        error2 = self.dll.AT_Close(self.handle)
        if error2 != 0:
            print('[AndorDevice] AT_Close error: ' + ERROR_CODE[error2])
        error3 = self.dll.AT_FinaliseLibrary('')
        if error3 != 0:
            print('[AndorDevice] AT_FinaliseLibrary error: ' + ERROR_CODE[error3])
        error4 = self.utildll.AT_FinaliseUtilityLibrary('')
        if error4 != 0:
            print('[AndorDevice] AT_FinaliseUtilityLibrary error: ' + ERROR_CODE[error4])
        if error1 + error2 + error3 + error4 > 0:
            error = -1
        else:
            error = 0
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]
        
    def GetCameraSerialNumber(self):
        max_lng = c_int()
        error = self.dll.AT_GetStringMaxLength(self.handle,'SerialNumber', byref(max_lng))
        if error != 0:
            print('[AndorDevice] AT_GetStringMaxLength error: ' + ERROR_CODE[error])
            max_lng = c_int(17)
        ser_num = create_string_buffer(max_lng.value)
        error = self.dll.AT_GetString(self.handle, 'SerialNumber', byref(ser_num))
        sn = ser_num.raw
        self.serial = sn.decode('utf-8')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetCycleMode(self,mode):
        error = self.dll.AT_SetEnumString(self.handle, 'CycleMode', mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetNumberAccumulations(self,number):
        acc_count = c_int(number)
        error = self.dll.AT_SetInt(self.handle, 'AccumulateCount', acc_count)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetFrameCount(self,number):
        fr_count = c_int(number)
        error = self.dll.AT_SetInt(self.handle, 'FrameCount', fr_count)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetShutter(self,mode):
        error = self.dll.AT_SetEnumString(self.handle, 'ShutterMode', mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetReadoutRate(self,rate):
        error = self.dll.AT_SetEnumString(self.handle, 'PixelReadoutRate', rate)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetHBin(self,hbin):
        self.hbin = hbin
        error = self.dll.AT_SetInt(self.handle, 'AOIHBin', c_int(self.hbin))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetVBin(self,vbin):
        self.vbin = vbin
        error = self.dll.AT_SetInt(self.handle, 'AOIVBin', c_int(self.vbin))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetHeight(self,height):
        self.height = height
        error = self.dll.AT_SetInt(self.handle, 'AOIHeight', c_int(self.height))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetWidth(self,width):
        self.width = width
        error = self.dll.AT_SetInt(self.handle, 'AOIWidth', c_int(self.width))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetAOILeft(self, left):
        self.left = left
        error = self.dll.AT_SetInt(self.handle, 'AOILeft', c_int(self.left))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetAOITop(self, top):
        self.top = top
        error = self.dll.AT_SetInt(self.handle, 'AOITop', c_int(self.top))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def StartAcquisition(self):
        error = self.dll.AT_QueueBuffer(self.handle, self.imageBufferPointer, self.buffer_size)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        if error != 0:
            print('[AndorDevice] Queue buffer error: ' + ERROR_CODE[error])
        # Start acquisition
        error = self.dll.AT_Command(self.handle, 'AcquisitionStart')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        if error != 0:
            print('[AndorDevice] Start acq error: ' + ERROR_CODE[error])
        # Wait for frame to be available
        error = self.dll.AT_WaitBuffer(self.handle, byref(self.imageBufferPointer), byref(self.im_size), 100000)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        if error != 0:
            print('[AndorDevice] Wait buffer error: ' + ERROR_CODE[error])
            # Try restarting camera
            print('[AndorDevice] Restarting...')
            error = self.Restart()
            if error != 0:
                return ERROR_CODE[error]
            else:
                error = -2
                return ERROR_CODE[error]
        # Stop acquisition
        error = self.dll.AT_Command(self.handle, 'AcquisitionStop')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        if error != 0:
            print('[AndorDevice] Stop acq error: ' + ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetImageSize(self):
        im_size = c_int()
        error = self.dll.AT_GetInt(self.handle, 'ImageSizeBytes', byref(im_size))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return im_size
    
    def GetStride(self):
        stride = c_int()
        error = self.dll.AT_GetInt(self.handle, 'AOIStride', byref(stride))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return stride.value

    # modified to just assign data to a pre-allocated array
    def GetAcquiredData2(self,imageArrayPointer):      
        stride = self.GetStride()
        #startTime = default_timer()                                     
        # Remove padding
        error = self.utildll.AT_ConvertBuffer(self.imageBufferPointer, imageArrayPointer, self.width, self.height, stride, u'Mono32', u'Mono32')
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        #endTime = default_timer()    
        #print("Andor conversion time = %.3f" % (endTime - startTime))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetAcquiredDataDim(self):
        dim = self.width * self.height / self.hbin / self.vbin
        return dim

    def SetExposureTime(self, time):
        self.exposure = time
        error = self.dll.AT_SetFloat(self.handle, 'ExposureTime', c_double(time))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetCoolerMode(self, mode):
        error = self.dll.AT_SetBool(self.handle, 'SensorCooling', c_bool(mode))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetTemperature(self):
        temp = c_double()
        error = self.dll.AT_GetFloat(self.handle, 'SensorTemperature', byref(temp))
        self.temperature = temp.value
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetTemperature(self, temp):
        # Cannot set temperature for Zyla camera
        return
        
    def SetFanMode(self, mode):
        ## Options:
        ## Off
        # On
        error = self.dll.AT_SetEnumString(self.handle, 'FanSpeed', mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SaveAsBmp(self, path):
        im=Image.new("RGB",(self.width,self.height),"white")
        pix = im.load()

        for i in range(len(self.imageArray)):
            (row, col) = divmod(i,self.width)
            picvalue = int(round(self.imageArray[i]))#*255.0/65535))
            pix[col,row] = (picvalue,picvalue,picvalue)

        im.save(path,"BMP")

    def SaveAsTxt(self, path):
        file = open(path, 'w')

        for line in self.imageArray:
            file.write("%g\n" % line)

        file.close()

    def SaveAsBmpNormalised(self, path):

        im=Image.new("RGB",(self.width,self.height),"white")
        pix = im.load()

        maxIntensity = max(self.imageArray)

        for i in range(len(self.imageArray)):
            (row, col) = divmod(i,self.width)
            picvalue = int(round(self.imageArray[i]*255.0/maxIntensity))
            pix[col,row] = (picvalue,picvalue,picvalue)

        im.save(path,"BMP")

    def SetPixelEncoding(self, setting):
        error = self.dll.AT_SetEnumString(self.handle, 'PixelEncoding', setting)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetPreAmpGain(self, setting):
        error = self.dll.AT_SetEnumString(self.handle, 'SimplePreAmpGainControl', setting)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def SetTriggerMode(self, mode):
        ## Options:
        ## Internal
        ## Software
        ## External
        ## External Start
        ## External Exposure
        error = self.dll.AT_SetEnumString(self.handle, 'TriggerMode', mode)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

ERROR_CODE = {
    -2: 'Restarted camera',
    -1: 'Failed to ShutDown',
    0: 'AT_SUCCESS',
    1: 'AT_ERR_NOTINITIALISED',
    2: 'AT_ERR_NOTIMPLEMENTED',
    3: 'AT_ERR_READONLY',
    4: 'AT_ERR_NOTREADABLE',
    5: 'AT_ERR_NOTWRITABLE',
    6: 'AT_ERR_OUTOFRANGE',
    7: 'AT_ERR_INDEXNOTAVAILABLE',
    8: 'AT_ERR_INDEXNOTIMPLEMENTED',
    9: 'AT_ERR_EXCEEDEDMAXSTRINGLENGTH',
    10: 'AT_ERR_CONNECTION',
    11: 'AT_ERR_NODATA',
    12: 'AT_ERR_INVALIDHANDLE',
    13: 'AT_ERR_TIMEDOUT',
    14: 'AT_ERR_BUFFERFULL',
    15: 'AT_ERR_INVALIDSIZE',
    16: 'AT_ERR_INVALIDALIGNMENT',
    17: 'AT_ERR_COMM',
    18:'AT_ERR_STRINGNOTAVAILABLE',
    19: 'AT_ERR_STRINGNOTIMPLEMENTED',
    20: 'AT_ERR_NULL_FEATURE',
    21: 'AT_ERR_NULL_HANDLE',
    22: 'AT_ERR_NULL_IMPLEMENTED_VAR',
    23: 'AT_ERR_NULL_READABLE_VAR',
    24: 'AT_ERR_NULL_READONLY_VAR',
    25: 'AT_ERR_NULL_WRITABLE_VAR',
    26: 'AT_ERR_NULL_MINVALUE',
    27: 'AT_ERR_NULL_MAXVALUE',
    28: 'AT_ERR_NULL_VALUE',
    29: 'AT_ERR_NULL_STRING',
    30: 'AT_ERR_NULL_COUNT_VAR',
    31: 'AT_ERR_NULL_ISAVAILABLE_VAR',
    32: 'AT_ERR_NULL_MAXSTRINGLENGTH',
    33: 'AT_ERR_NULL_EVCALLBACK',
    34: 'AT_ERR_NULL_QUEUE_PTR',
    35: 'AT_ERR_NULL_WAIT_PTR',
    36: 'AT_ERR_NULL_PTRSIZE',
    37: 'AT_ERR_NOMEMORY',
    100: 'AT_ERR_HARDWARE_OVERFLOW'
    }