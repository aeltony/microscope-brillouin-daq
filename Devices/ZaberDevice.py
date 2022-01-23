from __future__ import division
import Devices.BrillouinDevice
import time
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import numpy as np
import queue as Queue
import threading
import zaber.serial as zs

# Zaber motor does not need to run in its own thread, so we don't implement a getData() method
class ZaberDevice(Devices.BrillouinDevice.Device):

	# This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(ZaberDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Zaber"
        self.enqueueData = False
        self.commandQueue = Queue.Queue()
        self.homeLocX = 60000.
        self.homeLocY = 50000.
        self.homeLocZ = 6990.

        self.port = zs.AsciiSerial("COM4", baud=115200, timeout = 20, inter_char_timeout = 0.05)
        self.xy_device = zs.AsciiDevice(self.port, 3) # Sample stage
        self.z_device = zs.AsciiDevice(self.port, 4) # Objective stage
        self.y_axis = zs.AsciiAxis(self.xy_device, 1)
        self.x_axis = zs.AsciiAxis(self.xy_device, 2)
        self.z_axis = zs.AsciiAxis(self.z_device, 1)
        self.filter_device = zs.AsciiDevice(self.port, 5) # Filter wheel
        self.light_device = zs.AsciiDevice(self.port, 2)
        self.white_light = zs.AsciiAxis(self.light_device, 1) # Fluorescence light
        self.blue_light = zs.AsciiAxis(self.light_device, 2) # Fluorescence light
        self.red_light = zs.AsciiAxis(self.light_device, 3) # Fluorescence light
        self.trans_light = zs.AsciiAxis(self.light_device, 4) # Bright field light

        # Zero ('home') stages
        reply = self.z_axis.home()
        if self.checkReply(reply):
            print("[ZaberDevice] Z-axis homed")
        else:
            print("[ZaberDevice] Z-axis home failed")
        reply = self.x_axis.home()
        if self.checkReply(reply):
            print("[ZaberDevice] X-axis homed")
        else:
            print("[ZaberDevice] X-axis home failed")
        reply = self.y_axis.home()
        if self.checkReply(reply):
            print("[ZaberDevice] Y-axis homed")
        else:
            print("[ZaberDevice] Y-axis home failed")

        # Set filter wheel to brightfield setting (home, i.e. index 1)
        reply = self.filter_device.home()
        if self.checkReply(reply):
            print("[ZaberDevice] Filter wheel homed")
        else:
            print("[ZaberDevice] Filter wheel home failed")

        #print('xy status =', self.xy_device.get_status())
        #print('z status =', self.z_device.get_status())
        self.xy_microstep_size = 0.15625 #Microstep resolution in um
        self.z_microstep_size = 0.001 #Encoder count size in um

        # Get maximum current for all lamps
        reply = self.white_light.send("get lamp.current.max") # Max. current for white light
        self.whiteMaxCurrent = float(reply.data)
        print("[ZaberDevice] White light max. current = %.1f A" %self.whiteMaxCurrent)

        reply = self.blue_light.send("get lamp.current.max") # Max. current for blue light
        self.blueMaxCurrent = float(reply.data)
        print("[ZaberDevice] Blue light max. current = %.1f A" %self.blueMaxCurrent)

        reply = self.red_light.send("get lamp.current.max") # Max. current for red light
        self.redMaxCurrent = float(reply.data)
        print("[ZaberDevice] Red light max. current = %.1f A" %self.redMaxCurrent)

        reply = self.trans_light.send("get lamp.current.max") # Max. current for transmitted light
        self.transMaxCurrent = float(reply.data)
        print("[ZaberDevice] Transmitted light max. current = %.1f A" %self.transMaxCurrent)

        #speed = 26
        #speed_cmd = zs.AsciiCommand("set speed", int(speed/26*894455))  # 894455 = 26mm/s
        #self.x_axis.send(speed_cmd)
        #self.y_axis.send(speed_cmd)
        #self.z_axis.send(speed_cmd)
        #print("[ZaberDevice] Motor speed set to %f mm/s" % speed)

        #acceleration_cmd = zs.AsciiCommand("set accel", 600)
        #self.x_axis.send(acceleration_cmd)
        #self.y_axis.send(acceleration_cmd)
        #self.z_axis.send(acceleration_cmd)

        self.ZaberLock = app.ZaberLock
        self.updateLock = threading.Lock()
        self._lastPosition = [0, 0, 0]
        self.updatePosition('a')

        # Move stages to center location (scan home)
        self.moveHome('a')
        self.updatePosition('a')

        # Turn brightfield on, other lamps off:
        self.lightSwitch('white', False)
        self.lightSwitch('blue', False)
        self.lightSwitch('red', False)
        self.lightSwitch('trans', True)

    def shutdown(self):
        with self.ZaberLock:
            self.port.close()
        print("[ZaberDevice] Closed")

    # Zaber doesn't do any data acquisition. This method sends out commands from the command queue
    # so they can be called asynchronously. Right now don't have a way to Get data, only set
    # command should be a tuple of ('method name', argument)
    def getData(self):
        try:
            cmd = self.commandQueue.get(block=True, timeout=1.0)
            method = getattr(self, cmd[0])
            if len(cmd)==1:
                method()
            elif len(cmd)==2:
                method(cmd[1])
            elif len(cmd)==3:
                method(cmd[1], cmd[2])
        except Queue.Empty:
            pass

    # Checks for warnings + errors from Zaber devices
    def checkReply(self, reply):
        if reply.reply_flag != "OK":
            print ("[ZaberDevice] Command rejected because: {}".format(reply.data))
            return False
        else: # Command was accepted
            return True

    def setMotorAsync(self, methodName, whichAxis='a', arg=[]):
        if (arg == []):
            self.commandQueue.put((methodName, whichAxis))
        else:
            self.commandQueue.put((methodName, whichAxis, arg[0]))

    # returns current position of motor, in um
    def updatePosition(self, whichAxis='a'):
        #print('[ZaberDevice] updatePosition')
        with self.ZaberLock:
            if whichAxis == 'x':
                try:
                    reply_x = self.x_axis.send("get pos")
                except:
                    print("[ZaberDevice] Motor busy")
                    return self._lastPosition
            elif whichAxis == 'y':
                try:
                    reply_y = self.y_axis.send("get pos")
                except:
                    print("[ZaberDevice] Motor busy")
                    return self._lastPosition
            elif whichAxis == 'z':
                try:
                    reply_z = self.z_axis.send("get pos")
                except:
                    print("[ZaberDevice] Motor busy")
                    return self._lastPosition
            elif whichAxis == 'a':
                try:
                    reply_x = self.x_axis.send("get pos")
                    reply_y = self.y_axis.send("get pos")
                    reply_z = self.z_axis.send("get pos")
                except:
                    print("[ZaberDevice] Motor busy")
                    return self._lastPosition
        with self.updateLock:
            if whichAxis == 'x':
                self._lastPosition[0] = float(reply_x.data) * self.xy_microstep_size
            elif whichAxis == 'y':
                self._lastPosition[1] = float(reply_y.data) * self.xy_microstep_size
            elif whichAxis == 'z':
                self._lastPosition[2] = float(reply_z.data) * self.z_microstep_size
            elif whichAxis == 'a':
                self._lastPosition[0] = float(reply_x.data) * self.xy_microstep_size
                self._lastPosition[1] = float(reply_y.data) * self.xy_microstep_size
                self._lastPosition[2] = float(reply_z.data) * self.z_microstep_size
        return self._lastPosition

    # moves Zaber stages to home position
    def moveHome(self, whichAxis='a'):
        if whichAxis == 'x':
            self.moveAbs(whichAxis, self.homeLocX)
        elif whichAxis == 'y':
            self.moveAbs(whichAxis, self.homeLocY)
        elif whichAxis == 'z':
            self.moveAbs(whichAxis, self.homeLocZ)
        elif whichAxis == 'a':
            self.moveAbs('x', self.homeLocX)
            self.moveAbs('y', self.homeLocY)
            self.moveAbs('z', self.homeLocZ)

    # moves Zaber motor, called on by forwards and backwards buttons
    # distance in um
    def moveRelative(self, whichAxis='a', distance=0):
        # print "[ZaberDevice] moveRelative %f um" % distance
        with self.ZaberLock:
            if whichAxis == 'x':
                reply = self.x_axis.move_rel(int(distance/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] X-axis moveRelative failed")
            elif whichAxis == 'y':
                reply = self.y_axis.move_rel(int(distance/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Y-axis moveRelative failed")
            elif whichAxis == 'z':
                reply = self.z_axis.move_rel(int(distance/self.z_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Z-axis moveRelative failed")
            elif whichAxis == 'a':
                reply = self.x_axis.move_rel(int(distance/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] X-axis moveRelative failed")
                reply = self.y_axis.move_rel(int(distance/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Y-axis moveRelative failed")
                reply = self.z_axis.move_rel(int(distance/self.z_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Z-axis moveRelative failed")
        self.updatePosition(whichAxis)

    # moves Zaber motor to a set location, called on above
    def moveAbs(self, whichAxis='a', pos=0):
        #print("[ZaberDevice] moveAbs called with pos %d encoder counts" % int(pos/self.xy_microstep_size))
        with self.ZaberLock:
            if whichAxis == 'x':
                reply = self.x_axis.move_abs(int(pos/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] X-axis moveAbs failed")
            elif whichAxis == 'y':
                reply = self.y_axis.move_abs(int(pos/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Y-axis moveAbs failed")
            elif whichAxis == 'z':
                reply = self.z_axis.move_abs(int(pos/self.z_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Z-axis moveAbs failed")
            elif whichAxis == 'a':
                reply = self.x_axis.move_abs(int(pos/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] X-axis moveAbs failed")
                reply = self.y_axis.move_abs(int(pos/self.xy_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Y-axis moveAbs failed")
                reply = self.z_axis.move_abs(int(pos/self.z_microstep_size))
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Z-axis moveAbs failed")
        self.updatePosition(whichAxis)

    def moveFilter(self, index=1):
        # Index 1 = brightfield (BS)
        # Index 2 = fluorescence (dichroic)
        with self.ZaberLock:
            reply = self.filter_device.send("move index %d" %index)
            if self.checkReply(reply)==False:
                print("[ZaberDevice] Filter wheel move to index %d failed" %index)

    def lightSwitch(self, light='trans', value=True):
        # True = light ON, False = light OFF
        with self.ZaberLock:
            if light=='white':
                if value:
                    reply = self.white_light.send("lamp on")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] White light ON failed")
                else:
                    reply = self.white_light.send("lamp off")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] White light OFF failed")
            if light=='blue':
                if value:
                    reply = self.blue_light.send("lamp on")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Blue light ON failed")
                else:
                    reply = self.blue_light.send("lamp off")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Blue light OFF failed")
            if light=='red':
                if value:
                    reply = self.red_light.send("lamp on")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Red light ON failed")
                else:
                    reply = self.red_light.send("lamp off")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Red light OFF failed")
            if light=='trans':
                if value:
                    reply = self.trans_light.send("lamp on")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Transmitted light ON failed")
                else:
                    reply = self.trans_light.send("lamp off")
                    if self.checkReply(reply)==False:
                        print("[ZaberDevice] Transmitted light OFF failed")
                    
    def setPower(self, light='trans', percent=0):
        with self.ZaberLock:
            if light=='white':
                current = percent*self.whiteMaxCurrent/100
                reply = self.white_light.send("set lamp.current %f" %current)
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] White light set power failed")
            if light=='blue':
                current = percent*self.blueMaxCurrent/100
                reply = self.blue_light.send("set lamp.current %f" %current)
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Blue light set power failed")
            if light=='red':
                current = percent*self.redMaxCurrent/100
                reply = self.red_light.send("set lamp.current %f" %current)
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Red light set power failed")
            if light=='trans':
                current = percent*self.transMaxCurrent/100
                reply = self.trans_light.send("set lamp.current %f" %current)
                if self.checkReply(reply)==False:
                    print("[ZaberDevice] Transmitted light set power failed")

    def getPower(self, light='trans'):
        with self.ZaberLock:
            if light=='white':
                reply = self.white_light.send("get lamp.current")
                power = 100*float(reply.data)/self.whiteMaxCurrent # Percentage
            if light=='blue':
                reply = self.blue_light.send("get lamp.current")
                power = 100*float(reply.data)/self.blueMaxCurrent # Percentage
            if light=='red':
                reply = self.red_light.send("get lamp.current")
                power = 100*float(reply.data)/self.redMaxCurrent # Percentage
            if light=='trans':
                reply = self.trans_light.send("get lamp.current")
                power = 100*float(reply.data)/self.transMaxCurrent # Percentage
        return power
