import Devices.BrillouinDevice
import time
import struct
import serial
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

# This is the DS Instruments microwave source
# Microwave source does not need to run in its own thread, so we don't implement a getData() method
class SynthDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(SynthDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Synth"
        self.badCommand = b'[BADCOMMAND]\r\n'    # response if a command failed (b makes it into bytes)
        self.port = serial.Serial("COM3", 115200, timeout=10) #Change the COM PORT NUMBER to match your device
        if self.port.isOpen():    # make sure port is open
            self.port.write(b'*IDN?\n')   # send the standard SCPI identify command
            result = self.port.readline()
            print("[SynthDevice] Microwave source found: " + (result.strip()).decode('utf-8'))
        else:
            print('[SynthDevice] Could not open port')
        # Set initial RF power in dBm
        self.port.write(b'POWER +1.0dBm\n')
        self.port.write(b'POWER?\n')
        result = self.port.readline()
        print('[SynthDevice] RF power set to ' + (result.strip()).decode('utf-8'))
        # Set initial RF frequency in GHz
        self.port.write(b'FREQ:CW 5750MHz\n')
        self.port.write(b'FREQ:CW?\n')
        result = self.port.readline()
        print('[SynthDevice] RF frequency set to ' + (result.strip()).decode('utf-8'))
        # Enable RF output
        self.port.write(b'OUTP:STAT ON\n')
        self.port.write(b'OUTP:STAT?\n')
        result = self.port.readline()
        print('[SynthDevice] RF output is ' + (result.strip()).decode('utf-8'))
        self.synth_lock = app.synth_lock
        self.runMode = 0    #0 is free running, 1 is scan

    def shutdown(self):
        with self.synth_lock:
            self.port.write(b'OUTP:STAT OFF\n')
            time.sleep(0.1)
            self.port.close()
        print("[SynthDevice] Closed")

    def __del__(self):
        return 0
    
    def getFreq(self):
        #print('[SynthDevice] getFreq got called')
        with self.synth_lock:
            self.port.write(b'FREQ:CW?\n')  # try asking for signal generator setting
            result = self.port.readline()
            freq = float(result[:-4])*1e-9
        return freq

    def setFreq(self, freq):
        #print('[SynthDevice] setFreq got called with f =', freq)
        freq_MHz = freq*1e3
        command = b'FREQ:CW %.1f' % freq_MHz + b'MHz\n'
        self.port.write(command)
        #print("[SynthDevice] RF frequency set to %.3f GHz" % freq)

    def getPower(self):
        #print('[SynthDevice] getPower got called')
        with self.synth_lock:
            self.port.write(b'POWER?\n')
            result = self.port.readline()
            power = float(result[:-5])
            return power

    def setPower(self, power):
        #print('[SynthDevice] setPower got called with power =', power)
        command = b'POWER +%.1f' % power + b'dBm\n'
        self.port.write(command)
        #print("[SynthDevice] Power set to %.1f dBm" % power)