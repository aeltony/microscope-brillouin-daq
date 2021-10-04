import threading
import datetime
import numpy as np
import time
import math
import sys
import os
import ntpath
import csv
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
from pyqtgraphCustomize import *
import qt_ui # UI import
from ctypes import *
from configparser import ConfigParser
from Devices.SynthDevice import SynthDevice
from Devices.AndorDevice import AndorDevice, AndorProcessFreerun
from Devices.MakoDevice import MakoDevice, MakoFreerun
from Devices.TempSensorDevice import TempSensorDevice, TempSensorFreerun
from Devices.ZaberDevice import ZaberDevice
from Devices.ShutterDevice import ShutterDevice
from BrillouinScanManager import ScanManager
from ExperimentData import *
from BrillouinTreeModel import *

class CustomViewBox(pg.ViewBox):
    def __init__(self, *args, **kwds):
        pg.ViewBox.__init__(self, *args, **kwds)
        self.setMouseMode(self.RectMode)
        self.setBackgroundColor((255,255,255,0))
        
    ## reimplement right-click to zoom out
    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            self.autoRange(padding=0)
            
    def mouseDragEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            ev.ignore()
        else:
            pg.ViewBox.mouseDragEvent(self, ev)


class App(QtGui.QMainWindow,qt_ui.Ui_MainWindow):
 
    def __init__(self):
        super(App,self).__init__()
        self.setupUi(self)

        # load default/save parameter first
        self.configFilename = 'BrillouinScanConfig.ini'
        self.configParser = ConfigParser()
        self.configParser.read(self.configFilename)
        # TODO: validate configuration file, set defaults if configuration file is corrupted or unavailable
        self.maxScanPoints = 400  # Number of data points in view in freerunning mode
        self.maxRowPoints = 20 # Number of pixels per row in freerunning Brillouin map
        self.maxColPoints = 20 # Number of pixels per column in freerunning Brillouin map
        self.calPoints = 0 # Number of calibration points (used when scanning)
        self.xy_microstep_size = 0.15625 # Sample stage (Zaber) microstep resolution in um
        laserX = self.configParser.getfloat('More Settings', 'laser_position_x')
        laserY = self.configParser.getfloat('More Settings', 'laser_position_y')
        FSR = self.configParser.getfloat('More Settings', 'FSR')
        SD = self.configParser.getfloat('More Settings', 'SD')
        whitePower = self.configParser.getfloat('Lamps', 'white')
        bluePower = self.configParser.getfloat('Lamps', 'blue')
        redPower = self.configParser.getfloat('Lamps', 'red')
        transPower = self.configParser.getfloat('Lamps', 'trans')
        RFpower = self.configParser.getfloat('Synth', 'RF_power')
        spectColumn = self.configParser.getint('Andor', 'spect_column')
        spectRow = self.configParser.getint('Andor', 'spect_row')
        frameNumX = 20
        frameNumY = 20

        self.params = [
            {'name': 'Scan', 'type': 'group', 'children': [
                {'name': 'Step Size', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'float', 'value': 0.9375, 'suffix':' um', 'step': self.xy_microstep_size, 'limits':(0,1562.5),'decimals':5},
                    {'name': 'Y', 'type': 'float', 'value': 0.9375, 'suffix':' um', 'step': self.xy_microstep_size, 'limits':(0,1562.5),'decimals':5},
                    {'name': 'Z', 'type': 'float', 'value': 0, 'suffix':' um', 'step': 0.15, 'limits':(0,1500),'decimals':5}]},
                {'name': 'Frame Number', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'int', 'value': frameNumX, 'step': 1, 'limits':(1,2000)},
                    {'name': 'Y', 'type': 'int', 'value': frameNumY, 'step': 1, 'limits':(1,2000)},
                    {'name': 'Z', 'type': 'int', 'value': 1, 'step': 1, 'limits':(1,2000)}]},
                {'name': 'ToggleReference', 'type':'toggle', 'ButtonText':('Switch to Reference', 'Switch to Sample')}, #False=Sample="Switch to Ref"
                {'name': 'Scan/Cancel', 'type': 'action2', 'ButtonText':('Scan', 'Cancel')}
            ]},
            {'name': 'Display', 'type': 'group', 'children': [
                {'name': 'Colormap (min.)', 'type': 'float', 'value': 4.9, 'suffix':' GHz', 'step': 0.01, 'limits':(0.0, 22.0), 'decimals':3},
                {'name': 'Colormap (max.)', 'type': 'float', 'value': 5.6, 'suffix':' GHz', 'step': 0.01, 'limits':(0.1, 22.0), 'decimals':3}
            ]},
            {'name': 'Motor', 'type': 'group', 'children': [
                {'name': 'Curr. X location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':6},
                {'name': 'Curr. Y location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':6},
                {'name': 'Curr. Z location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':6},
                {'name': 'Jog step', 'type': 'float', 'value': 1, 'suffix':' um', 'step': 1, 'limits':(0, 5000)},
                {'name': 'Jog X', 'type': 'action3', 'ButtonText':('Jog X +', 'Jog X -', 'Home X')},
                {'name': 'Jog Y', 'type': 'action3', 'ButtonText':('Jog Y +', 'Jog Y -', 'Home Y')},
                {'name': 'Jog Z', 'type': 'action3', 'ButtonText':('Jog Z +', 'Jog Z -', 'Home Z')},
                {'name': 'Move to location', 'type': 'float', 'value':0, 'suffix':' um', 'limits':(0, 100000), 'decimals':5},
                {'name': 'Move', 'type':'action3', 'ButtonText':('Move X', 'Move Y', 'Move Z')}
            ]},
            {'name': 'Spectrometer Camera', 'type': 'group', 'children': [
                {'name': 'Background Subtraction', 'type':'toggle', 'ButtonText':('Turn on background subtraction', 'Turn off background subtraction')},
                {'name': 'Exposure', 'type':'float', 'value':0.1, 'suffix':' s', 'step':0.01, 'limits':(0.001, 60)},
                {'name': 'Ref. Exposure', 'type':'float', 'value':0.1, 'suffix':' s', 'step':0.01, 'limits':(0.001, 10)},
                #{'name': 'AutoExposure', 'type':'toggle', 'ButtonText':('Auto exposure', 'Fixed exposure')}, #False=Fixed exposure
                #{'name': 'Camera Temp.', 'type': 'float', 'value':0, 'suffix':' C', 'readonly': True}
            ]},
            {'name': 'Microscope Camera', 'type': 'group', 'children': [
                {'name': 'Brightfield Exp.', 'type': 'float', 'value': 20, 'suffix':' ms', 'limits':(0.001, 20000)},
                {'name': 'Fluoresc. Exp.', 'type': 'float', 'value': 200, 'suffix':' ms', 'limits':(0.001, 20000)},
                {'name': 'White Light', 'type': 'float', 'value': whitePower, 'suffix':' %', 'step': 1, 'limits': (0, 100), 'decimals':3},
                {'name': 'Blue Light', 'type': 'float', 'value': bluePower, 'suffix':' %', 'step': 1, 'limits': (0, 100), 'decimals':3},
                {'name': 'Red Light', 'type': 'float', 'value': redPower, 'suffix':' %', 'step': 1, 'limits': (0, 100), 'decimals':3},
                {'name': 'Trans. Light', 'type': 'float', 'value': transPower, 'suffix':' %', 'step': 1, 'limits': (0, 100), 'decimals':3},
                {'name': 'Magnification', 'type': 'float', 'value': 20, 'suffix':' X', 'limits':(0.01, 200)},
                #{'name': 'Frame Rate', 'type': 'int', 'value': 3, 'suffix':' Hz', 'limits':(1, 20)},
                {'name': 'ToggleFluorescence', 'type':'toggle', 'ButtonText':('Switch to Fluorescence', 'Switch to Brightfield')} #False=Brightfield="Switch to Fluorescence"
            ]},
            {'name': 'Microwave Source', 'type': 'group', 'children': [
                {'name': 'Cal. Freq (min.)', 'type': 'float', 'value': 4.9, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3},
                {'name': 'Cal. Freq (max.)', 'type': 'float', 'value': 5.9, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3},
                {'name': 'Cal. Freq (step)', 'type': 'float', 'value': 0.35, 'suffix':' GHz', 'step': 0.001, 'limits': (0.001, 13.0), 'decimals':3},
                {'name': 'RF Power', 'type': 'float', 'value': RFpower, 'suffix':' dBm', 'step': 0.1, 'limits': (-20, 10), 'decimals':2},
                {'name': 'RF Frequency', 'type': 'float', 'value': 5.5, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3}
            ]},
            {'name': 'More Settings', 'type': 'group', 'children': [
                {'name': 'Ambient Temp.', 'type': 'float', 'value': 0.0, 'suffix':' deg. C', 'readonly':True, 'decimals':4},
                {'name': 'FSR', 'type': 'float', 'value': FSR, 'suffix':' GHz', 'limits':(5, 100), 'decimals':5},
                {'name': 'SD', 'type': 'float', 'value': SD, 'suffix':' GHz/px', 'limits':(0, 2), 'decimals':4},
                {'name': 'Spectrum Col.', 'type':'int', 'value': spectColumn, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Spectrum Row', 'type':'int', 'value': spectRow, 'suffix':' px', 'step': 1, 'limits':(0, 2048)},
                {'name': 'Laser Focus X', 'type': 'float', 'value': laserX, 'suffix':' um', 'step': 0.1, 'limits':(-5000.0, 5000.0),'decimals':3},
                {'name': 'Laser Focus Y', 'type': 'float', 'value': laserY, 'suffix':' um', 'step': 0.1, 'limits':(-5000.0, 5000.0),'decimals':3}
            ]}
        ]
        ## Create tree of Parameter objects
        self.allParameters = Parameter.create(name='params', type='group', children=self.params)
        self.parameterTreeWidget.setParameters(self.allParameters, showTop=False)

        self.session = None
        self.dataFile = None
        self.dataFileName = None
        # self.activeExperiment = None

        #Lock used to halt other threads upon app closing
        self.stop_event = threading.Event()
        self.synth_lock = threading.Lock()
        self.andor_lock = threading.Lock()
        self.mako_lock = threading.Lock()
        self.TempSensor_lock = threading.Lock()
        self.ZaberLock = threading.Lock()
        self.scan_lock = threading.Lock()

        # Even though ZaberDevice is a QThread, for now we don't need to run it in a separate thread
        # It is implemented this way for future-proofing
        self.ZaberDevice = ZaberDevice(self.stop_event, self)
        self.ZaberDevice.start()

        self.TempSensorDeviceThread = TempSensorDevice(self.stop_event, self)
        self.TempSensorProcessThread = TempSensorFreerun(self.TempSensorDeviceThread, self.stop_event)
        self.TempSensorProcessThread.updateTempSeqSig.connect(self.UpdateAmbientTemp)
        self.TempSensorDeviceThread.start()
        self.TempSensorProcessThread.start()

        self.SynthDevice = SynthDevice(self.stop_event, self)
        self.SynthDevice.start()

        self.ShutterDevice = ShutterDevice(self)    # Initialized to SAMPLE_STATE

        self.MakoDeviceThread = MakoDevice(self.stop_event, self)
        self.MakoProcessThread = MakoFreerun(self.MakoDeviceThread, self.stop_event)
        self.MakoProcessThread.updateCMOSImageSig.connect(self.MakoProcessUpdate)
        self.MakoDeviceThread.start()
        self.MakoProcessThread.start()

        self.AndorDeviceThread = AndorDevice(self.stop_event, self)
        self.AndorProcessThread = AndorProcessFreerun(self.AndorDeviceThread, self.stop_event)
        self.AndorProcessThread.updateSampleImageSig.connect(self.AndorSampleProcessUpdate)
        self.AndorProcessThread.updateBinnedImageSig.connect(self.AndorBinnedProcessUpdate)

        # self.CMOSview = CustomViewBox(invertY=True)             # The ViewBox is a zoomable/pannable box
        self.CMOSview = pg.PlotItem()
        self.graphicsViewCMOS.setCentralItem(self.CMOSview)     # GraphicsView is the main graphics container
        self.CMOSview.setAspectLocked(True)
        self.pxWid = self.MakoDeviceThread.imageWidth//self.MakoDeviceThread.bin_size
        self.pxHt = self.MakoDeviceThread.imageHeight//self.MakoDeviceThread.bin_size
        self.CMOSImage = pg.ImageItem(np.zeros((self.pxWid, self.pxHt)))      # ImageItem contains what needs to be plotted
        self.scaleFactor = 4.5/self.allParameters.child('Microscope Camera').child('Magnification').value() # 4.5 um/px
        self.CMOSImage.translate(-0.5*self.pxWid*self.scaleFactor, -0.5*self.pxHt*self.scaleFactor)
        self.CMOSImage.scale(self.scaleFactor, self.scaleFactor)
        self.CMOSview.addItem(self.CMOSImage)
        self.CMOSview.autoRange(padding=0)                      # gets rid of padding
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, frameNumX-1, frameNumY-1)
        self.scanRegion = pg.PlotDataItem(scanRegX, scanRegY)
        self.scanRegion.setPen(width=1, color='r')
        self.CMOSview.addItem(self.scanRegion)
        #self.CMOSvLine = pg.InfiniteLine(angle=90, movable=False)
        #self.CMOShLine = pg.InfiniteLine(angle=0, movable=False)
        #self.CMOSview.addItem(self.CMOSvLine, ignoreBounds=True)
        #self.CMOSview.addItem(self.CMOShLine, ignoreBounds=True)
        #self.CMOSvLine.setPos(laserX)
        #self.CMOShLine.setPos(laserY)

        # Import standard Brillouin colormap data
        arr = []
        with open('Utilities\\colormap.txt', 'r') as f:
            reader = csv.reader(f, delimiter='\t')
            for row in reader:
                arr.append([float(x) for x in row] + [1]) # RGBA, 0.0 to 1.0
        self.heatmapColormapArray = np.array(arr)
        self.colormap = pg.ColorMap(np.linspace(0, 1, len(self.heatmapColormapArray)), self.heatmapColormapArray)

        # Data acquisition Heatmap
        self.heatmapScanData = np.array([])
        self.blankHeatmap = np.zeros((self.maxRowPoints, self.maxColPoints))
        self.heatmapPlot = pg.PlotItem()
        self.heatmapPlot.setAspectLocked(False)
        self.graphicsViewHeatmap.setCentralItem(self.heatmapPlot)     # GraphicsView is the main graphics container
        self.heatmapImage = pg.ImageItem(self.blankHeatmap)
        self.heatmapImage.setLookupTable(self.colormap.getLookupTable())
        self.heatmapPlot.addItem(self.heatmapImage)
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)
        self.heatmapPlot.autoRange(padding=0)

        # Andor Live image - Sample
        self.SampleView = CustomViewBox(invertY=True)
        self.graphicsViewSample.setCentralItem(self.SampleView)
        self.SampleView.setAspectLocked(True)
        self.SampleImage = pg.ImageItem(np.zeros((860, 200)))
        self.SampleView.addItem(self.SampleImage)
        self.SampleView.autoRange(padding=0)

         # Andor Live image - Binned
        self.BinnedView = CustomViewBox(invertY=True)
        self.graphicsViewBinned.setCentralItem(self.BinnedView)
        self.BinnedView.setAspectLocked(True)
        self.BinnedImage = pg.ImageItem(np.zeros((440, 100)))
        self.BinnedView.addItem(self.BinnedImage)
        self.BinnedView.autoRange(padding=0)

        # sampleSpectrumPlot is the plot of the raw data and Lorentzian fit
        self.sampleSpectrumPlot = pg.PlotItem()
        # self.sampleSpectrumPlot.setYRange(0, 17500)
        self.sampleSpectrumPlot.enableAutoRange(axis=self.sampleSpectrumPlot.vb.YAxis, enable=True)
        self.graphicsViewSampleSpectrum.setCentralItem(self.sampleSpectrumPlot)
        self.sampleSpectrumItem = pg.PlotDataItem(symbol='o')
        self.sampleSpectrumItem.setSymbolPen(color='g')
        self.sampleSpectrumItem.setPen(None)
        self.sampleSpectrumItem.setData(100*np.ones(100))
        self.sampleSpectrumItem2 = pg.PlotDataItem()
        self.sampleSpectrumItem2.setPen(width=2.5, color='r')
        self.sampleSpectrumItem2.setData(200*np.ones(100))
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem)
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem2)
        axBottom = self.sampleSpectrumPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpectrumPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # sampleScanDepthPlot is the Brillouin vs z axis plot
        self.sampleScanDepthPlot = pg.PlotItem()
        self.sampleScanDepthPlot.setYRange(4.95,5.95)
        self.graphicsViewSampleScanDepth.setCentralItem(self.sampleScanDepthPlot)
        self.sampleScanDepthPlot.enableAutoRange(axis=self.sampleScanDepthPlot.vb.XAxis, enable=True)
        self.sampleScanDepthItem = pg.PlotDataItem() 
        self.sampleScanDepthItem.setPen(width=2.5, color='g')
        self.sampleScanDepthPlot.addItem(self.sampleScanDepthItem)
        axBottom = self.sampleScanDepthPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleScanDepthPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        self.sampleScanDepthData = np.array([])

        # This is the spectrograph:
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.sampleSpecSeriesPlot = pg.PlotItem()
        self.graphicsViewSampleSpecSeries.setCentralItem(self.sampleSpecSeriesPlot)
        self.sampleSpecSeriesImage = pg.ImageItem(self.sampleSpecSeriesData)
        self.sampleSpecSeriesPlot.addItem(self.sampleSpecSeriesImage)
        axBottom = self.sampleSpecSeriesPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpecSeriesPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        self.mainUI()

        self.AndorDeviceThread.start()
        self.AndorDeviceThread.setPriority(QtCore.QThread.TimeCriticalPriority)
        self.AndorProcessThread.start()
        time.sleep(0.1)   # CRUCIAL!! DON'T REMOVE. Need to let the threads fully start before continuing

        # create the figures for plotting Brillouin shifts and fits
        self.AndorProcessThread.updateSampleBrillouinSeqSig.connect(self.UpdateSampleBrillouinSeqPlot)
        self.AndorProcessThread.updateSampleSpectrum.connect(self.UpdateSampleSpectrum)

        self.BrillouinScan = ScanManager(self.stop_event, self.ZaberDevice, self.ShutterDevice, self.SynthDevice)
        self.BrillouinScan.addToSequentialList(self.AndorDeviceThread, self.AndorProcessThread)
        self.BrillouinScan.addToPartialList(self.MakoDeviceThread, self.MakoProcessThread)
        self.BrillouinScan.addToPartialList(self.TempSensorDeviceThread, self.TempSensorProcessThread)
        self.BrillouinScan.finished.connect(self.onFinishScan)
        self.BrillouinScan.clearGUISig.connect(self.clearGUIElements)

        # Apply settings from config file
        self.AndorDeviceThread.setTopPx(int(spectColumn))
        self.AndorDeviceThread.setLeftPx(int(spectRow))
        self.ZaberDevice.setPower('white', whitePower)
        self.ZaberDevice.setPower('blue', bluePower)
        self.ZaberDevice.setPower('red', redPower)
        self.ZaberDevice.setPower('trans', transPower)
        self.SynthDevice.setPower(RFpower)

        self.InitHardwareParameterTree()

    #############################################################################################
    # This next group of methods are used to set/get hardware settings, using the parameterTree #
    #############################################################################################
    def InitHardwareParameterTree(self):
        # print("[InitHardwareParameterTree]")

        # ========================= Spectrometer Camera =============================
        pItem = self.allParameters.child('Spectrometer Camera')
        pItem.child('Background Subtraction').sigActivated.connect(self.bgSubtraction)
        #pItem.child('AutoExposure').sigActivated.connect(self.switchAutoExp)
        pItem.child('Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Ref. Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Exposure').setValue(self.AndorDeviceThread.getExposure())
        #pItem.child('Camera Temp.').setValue(self.AndorDeviceThread.getTemperature())

        # ========================= Brightfield Camera ==================================
        pItem = self.allParameters.child('Microscope Camera')
        pItem.child('Brightfield Exp.').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.MakoDeviceThread.setExpTime))
        pItem.child('Fluoresc. Exp.').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.MakoDeviceThread.setExpTime))
        pItem.child('ToggleFluorescence').sigActivated.connect(self.toggleFluorescence)
        pItem.child('White Light').sigValueChanged.connect(self.whiteLightPowerChange)
        pItem.child('White Light').setValue(self.ZaberDevice.getPower('white'))
        pItem.child('Blue Light').sigValueChanged.connect(self.blueLightPowerChange)
        pItem.child('Blue Light').setValue(self.ZaberDevice.getPower('blue'))
        pItem.child('Red Light').sigValueChanged.connect(self.redLightPowerChange)
        pItem.child('Red Light').setValue(self.ZaberDevice.getPower('red'))
        pItem.child('Trans. Light').sigValueChanged.connect(self.transLightPowerChange)
        pItem.child('Trans. Light').setValue(self.ZaberDevice.getPower('trans'))
        pItem.child('Magnification').sigValueChanged.connect(self.magnificationChange)
        #pItem.child('Frame Rate').sigValueChanged.connect(
        #    lambda data: self.changeHardwareSetting(data, self.MakoDeviceThread.setFrameRate))

        # ========================= Microwave Source ================================
        pItem = self.allParameters.child('Microwave Source')
        pItem.child('RF Frequency').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.SynthDevice.setFreq))
        pItem.child('RF Frequency').setValue(self.SynthDevice.getFreq())
        pItem.child('RF Power').sigValueChanged.connect(self.synthPowerValueChange)
        pItem.child('RF Power').setValue(self.SynthDevice.getPower())

        # ========================= Motor ===========================================
        pItem = self.allParameters.child('Motor')
        self.MotorPositionUpdate()
        
        # connect Jog Buttons
        motorFwdFunX = lambda: self.ZaberDevice.moveRelative( 
            'x', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunX = lambda: self.ZaberDevice.moveRelative( 
            'x', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog X').sigActivated.connect(motorFwdFunX)
        pItem.child('Jog X').sigActivated2.connect(motorBackFunX)
        pItem.child('Jog X').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'x'))

        motorFwdFunY = lambda: self.ZaberDevice.moveRelative( 
            'y', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunY = lambda: self.ZaberDevice.moveRelative( 
            'y', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog Y').sigActivated.connect(motorFwdFunY)
        pItem.child('Jog Y').sigActivated2.connect(motorBackFunY)
        pItem.child('Jog Y').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'y'))

        motorFwdFunZ = lambda: self.ZaberDevice.moveRelative( 
            'z', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunZ = lambda: self.ZaberDevice.moveRelative( 
            'z', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog Z').sigActivated.connect(motorFwdFunZ)
        pItem.child('Jog Z').sigActivated2.connect(motorBackFunZ)
        pItem.child('Jog Z').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'z'))

        # connect Move and Home buttons
        motorMoveFunX = lambda: self.ZaberDevice.moveAbs(
            'x', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated.connect(motorMoveFunX)
        motorMoveFunY = lambda: self.ZaberDevice.moveAbs(
            'y', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated2.connect(motorMoveFunY)
        motorMoveFunZ = lambda: self.ZaberDevice.moveAbs(
            'z', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated3.connect(motorMoveFunZ)

        # ========================= Scan ===========================================
        pItem = self.allParameters.child('Scan')
        pItem.child('Step Size').child('X').sigValueChanged.connect(self.stepSizeValueChangeX)
        pItem.child('Step Size').child('Y').sigValueChanged.connect(self.stepSizeValueChangeY)
        pItem.child('Frame Number').child('X').sigValueChanged.connect(self.frameNumValueChangeX)
        pItem.child('Frame Number').child('Y').sigValueChanged.connect(self.frameNumValueChangeY)
        pItem.child('Scan/Cancel').sigActivated.connect(self.startScan)
        pItem.child('Scan/Cancel').sigActivated2.connect(self.cancelScan)
        pItem.child('ToggleReference').sigActivated.connect(self.toggleReference)

        # ========================= More Settings ==================================
        pItem = self.allParameters.child('More Settings')
        pItem.child('FSR').sigValueChanged.connect(self.FSRValueChange)
        pItem.child('SD').sigValueChanged.connect(self.SDValueChange)
        # Spectrum Column / Row adjustment
        pItem.child('Spectrum Col.').sigValueChanged.connect(self.spectColumnValueChange)
        pItem.child('Spectrum Row').sigValueChanged.connect(self.spectRowValueChange)
        # Laser crosshair adjustment
        pItem.child('Laser Focus X').sigValueChanged.connect(self.CMOSvLineValueChange)
        pItem.child('Laser Focus Y').sigValueChanged.connect(self.CMOShLineValueChange)

        # ========================= Hardware monitor timers ========================

        #self.hardwareGetTimer = QTimer()
        #self.hardwareGetTimer.timeout.connect(self.HardwareParamUpdate)
        #self.hardwareGetTimer.start(600000)

        # Separate timer for motor position, update at faster rate
        self.motorPositionTimer = QTimer()
        self.motorPositionTimer.timeout.connect(self.MotorPositionUpdate)
        self.motorPositionTimer.start(500)

    def changeHardwareSetting(self, data, funcHandle):
        # print("[changeHardwareSetting]")
        funcHandle(data.value())

    #def HardwareParamUpdate(self):
    #    #print("[HardwareParamUpdate]")
    #    try:
    #        temp = self.AndorDeviceThread.getTemperature()
    #        self.allParameters.child('Spectrometer Camera').child('Camera Temp.').setValue(temp)
    #    except:
    #        print('Could not update AndorDevice temperature')

    @QtCore.pyqtSlot(list)
    def MotorPositionUpdate2(self, pos):
        #print("[MotorPositionUpdate2]")
        self.allParameters.child('Motor').child('Curr. X location').setValue(pos[0])
        self.allParameters.child('Motor').child('Curr. Y location').setValue(pos[1])
        self.allParameters.child('Motor').child('Curr. Z location').setValue(pos[2])

    def MotorPositionUpdate(self):
        #print("[MotorPositionUpdate]")
        pos = self.ZaberDevice.updatePosition()
        self.allParameters.child('Motor').child('Curr. X location').setValue(pos[0])
        self.allParameters.child('Motor').child('Curr. Y location').setValue(pos[1])
        self.allParameters.child('Motor').child('Curr. Z location').setValue(pos[2])

    def drawSquare(self, laserX, laserY, lenX, lenY):
        ptsPerSide = 10
        x = np.linspace(laserX, laserX + lenX, num=ptsPerSide, endpoint=True)
        y = np.linspace(laserY, laserY - lenY, num=ptsPerSide, endpoint=True)
        scanRegX = np.concatenate((x, (laserX + lenX)*np.ones(ptsPerSide), np.flip(x), laserX*np.ones(ptsPerSide)))
        scanRegY = np.concatenate((laserY*np.ones(ptsPerSide), y, (laserY - lenY)*np.ones(ptsPerSide), np.flip(y)))
        return (scanRegX, scanRegY)

    def stepSizeValueChangeX(self, param, value):
        corrStepSizeX = self.xy_microstep_size*int(value/self.xy_microstep_size) # correct for quantization of steps
        laserX = self.allParameters.child('More Settings').child('Laser Focus X').value()
        laserY = self.allParameters.child('More Settings').child('Laser Focus Y').value()
        lenX = corrStepSizeX*(self.allParameters.child('Scan').child('Frame Number').child('X').value()-1)
        lenY = self.allParameters.child('Scan').child('Step Size').child('Y').value()*(self.allParameters.child('Scan').child('Frame Number').child('Y').value()-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)
        self.allParameters.child('Scan').child('Step Size').child('X').setValue(corrStepSizeX) # update display to show quantized step

    def stepSizeValueChangeY(self, param, value):
        corrStepSizeY = self.xy_microstep_size*int(value/self.xy_microstep_size) # correct for quantization of steps
        laserX = self.allParameters.child('More Settings').child('Laser Focus X').value()
        laserY = self.allParameters.child('More Settings').child('Laser Focus Y').value()
        lenX = self.allParameters.child('Scan').child('Step Size').child('X').value()*(self.allParameters.child('Scan').child('Frame Number').child('X').value()-1)
        lenY = corrStepSizeY*(self.allParameters.child('Scan').child('Frame Number').child('Y').value()-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)
        self.allParameters.child('Scan').child('Step Size').child('Y').setValue(corrStepSizeY) # update display to show quantized step

    def frameNumValueChangeX(self, param, value):
        laserX = self.allParameters.child('More Settings').child('Laser Focus X').value()
        laserY = self.allParameters.child('More Settings').child('Laser Focus Y').value()
        lenX = self.allParameters.child('Scan').child('Step Size').child('X').value()*(value-1)
        lenY = self.allParameters.child('Scan').child('Step Size').child('Y').value()*(self.allParameters.child('Scan').child('Frame Number').child('Y').value()-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)

    def frameNumValueChangeY(self, param, value):
        laserX = self.allParameters.child('More Settings').child('Laser Focus X').value()
        laserY = self.allParameters.child('More Settings').child('Laser Focus Y').value()
        lenX = self.allParameters.child('Scan').child('Step Size').child('X').value()*(self.allParameters.child('Scan').child('Frame Number').child('X').value()-1)
        lenY = self.allParameters.child('Scan').child('Step Size').child('Y').value()*(value-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)

    def CMOShLineValueChange(self, param, value):
        # print("[CMOShLineValueChange]")
        #self.CMOShLine.setPos(value)
        laserX = self.allParameters.child('More Settings').child('Laser Focus X').value()
        laserY = value
        lenX = self.allParameters.child('Scan').child('Step Size').child('X').value()*(self.allParameters.child('Scan').child('Frame Number').child('X').value()-1)
        lenY = self.allParameters.child('Scan').child('Step Size').child('Y').value()*(self.allParameters.child('Scan').child('Frame Number').child('Y').value()-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)
        self.configParser.set('More Settings', 'laser_position_y', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def CMOSvLineValueChange(self, param, value):
        # print("[CMOSvLineValueChange]")
        #self.CMOSvLine.setPos(value)
        laserX = value
        laserY = self.allParameters.child('More Settings').child('Laser Focus Y').value()
        lenX = self.allParameters.child('Scan').child('Step Size').child('X').value()*(self.allParameters.child('Scan').child('Frame Number').child('X').value()-1)
        lenY = self.allParameters.child('Scan').child('Step Size').child('Y').value()*(self.allParameters.child('Scan').child('Frame Number').child('Y').value()-1)
        (scanRegX, scanRegY) = self.drawSquare(laserX, laserY, lenX, lenY)
        self.scanRegion.setData(scanRegX, scanRegY)
        self.configParser.set('More Settings', 'laser_position_x', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def magnificationChange(self):
        # First, reset offset + scaling
        self.CMOSImage.scale(1/self.scaleFactor, 1/self.scaleFactor)
        self.CMOSImage.translate(0.5*self.pxWid*self.scaleFactor, 0.5*self.pxHt*self.scaleFactor)
        # Then, update offset + scale
        self.scaleFactor = 4.5/self.allParameters.child('Microscope Camera').child('Magnification').value() # 4.5 um/px
        self.CMOSImage.translate(-0.5*self.pxWid*self.scaleFactor, -0.5*self.pxHt*self.scaleFactor)
        self.CMOSImage.scale(self.scaleFactor, self.scaleFactor)
        self.CMOSview.autoRange(padding=0)

    def synthPowerValueChange(self, param, value):
        self.SynthDevice.setPower(value)
        self.configParser.set('Synth', 'RF_power', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def whiteLightPowerChange(self, param, value):
        self.ZaberDevice.setPower('white', value)
        self.configParser.set('Lamps', 'white', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def blueLightPowerChange(self, param, value):
        self.ZaberDevice.setPower('blue', value)
        self.configParser.set('Lamps', 'blue', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def redLightPowerChange(self, param, value):
        self.ZaberDevice.setPower('red', value)
        self.configParser.set('Lamps', 'red', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def transLightPowerChange(self, param, value):
        self.ZaberDevice.setPower('trans', value)
        self.configParser.set('Lamps', 'trans', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def FSRValueChange(self, param, value):
        self.configParser.set('More Settings', 'FSR', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def SDValueChange(self, param, value):
        self.configParser.set('More Settings', 'SD', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def spectColumnValueChange(self, param, value):
        self.AndorDeviceThread.setTopPx(int(value))
        self.configParser.set('Andor', 'spect_column', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def spectRowValueChange(self, param, value):
        self.AndorDeviceThread.setLeftPx(int(value))
        self.configParser.set('Andor', 'spect_row', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    @QtCore.pyqtSlot()
    def clearGUIElements(self):
        # print("[clearGUIElements]")
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.sampleScanDepthData = np.array([])
        self.heatmapScanData = np.array([])

    def startScan(self):
        # First check that a session is running, and that an experiment is selected
        if self.session is None:
            choice = QtGui.QMessageBox.warning(self, 'Starting Scan...',
                                                'No Session open!',
                                                QtGui.QMessageBox.Ok)
            return

        print('Starting a scan in Exp_%d: ' % self.model.activeExperiment)

        # Check if 'Screenshots' directory exists, and if not, create one
        dataPath = os.path.dirname(self.dataFileName) + '\\Screenshots\\'
        if not os.path.exists(dataPath):
            os.makedirs(dataPath)

        stepSizeArr = np.array([self.allParameters.child('Scan').child('Step Size').child('X').value(), \
            self.allParameters.child('Scan').child('Step Size').child('Y').value(), \
            self.allParameters.child('Scan').child('Step Size').child('Z').value()])
        frameNumArr = np.array([self.allParameters.child('Scan').child('Frame Number').child('X').value(), \
            self.allParameters.child('Scan').child('Frame Number').child('Y').value(), \
            self.allParameters.child('Scan').child('Frame Number').child('Z').value()])
        calFreq = np.arange(self.allParameters.child('Microwave Source').child('Cal. Freq (min.)').value(), \
            self.allParameters.child('Microwave Source').child('Cal. Freq (max.)').value() + \
            self.allParameters.child('Microwave Source').child('Cal. Freq (step)').value(), \
            self.allParameters.child('Microwave Source').child('Cal. Freq (step)').value())
        magnification = self.allParameters.child('Microscope Camera').child('Magnification').value()

        flattenedParamList = generateParameterList(self.params, self.allParameters)

        # Check if scan should include fluorescence:
        if (self.allParameters.child('Microscope Camera').child('ToggleFluorescence').value() == True):
            takeFluorescence = True
        else:
            takeFluorescence = False

        scanSettings = {'step': stepSizeArr,
            'frames': frameNumArr,
            'calFreq': calFreq,
            'magnification': magnification,
            'laserX': self.allParameters.child('More Settings').child('Laser Focus X').value(),
            'laserY': self.allParameters.child('More Settings').child('Laser Focus Y').value(),
            'refExp': self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value(),
            'sampleExp': self.allParameters.child('Spectrometer Camera').child('Exposure').value(),
            'brightExp': self.allParameters.child('Microscope Camera').child('Brightfield Exp.').value(),
            'fluorExp': self.allParameters.child('Microscope Camera').child('Fluoresc. Exp.').value(),
            'dataPath' : dataPath,
            'takeFluorescence': takeFluorescence,
            'flattenedParamList': flattenedParamList }
        self.BrillouinScan.assignScanSettings(scanSettings)
        # Scale plot window to scan length (+ calFreq calibration frames per y-z coordinate)
        self.calPoints = calFreq.shape[0]
        self.maxScanPoints = frameNumArr[0] + self.calPoints
        self.maxRowPoints = frameNumArr[0] + self.calPoints
        self.maxColPoints = frameNumArr[1]
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0

        self.BrillouinScan.sessionData = self.session
        self.BrillouinScan.saveExpIndex = self.model.activeExperiment

        # Stop periodic polling of hardware state
        #self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()
        self.BrillouinScan.motorPosUpdateSig.connect(self.MotorPositionUpdate2)

        # Finally start scan in a new thread
        self.BrillouinScan.start()

    def cancelScan(self):
        print('Scan cancelled!')
        self.BrillouinScan.Cancel_Flag = True

    def onFinishScan(self):
        self.maxScanPoints = 400 # Re-scale plot window for free-running mode
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.maxRowPoints = 20
        self.maxColPoints = 20
        self.calPoints = 0
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)
        # Return microwave source to previous (non-calibration) set point
        self.SynthDevice.setFreq(self.allParameters.child('Microwave Source').child('RF Frequency').value())
        self.SynthDevice.setPower(self.allParameters.child('Microwave Source').child('RF Power').value())
        # Make sure switch matches position at end of scan (brightfield)
        self.allParameters.child('Microscope Camera').child('ToggleFluorescence').setValue(False)
        if (self.allParameters.child('Scan').child('ToggleReference').value() == True):
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorDeviceThread.setRefState(True)
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorDeviceThread.setRefState(False)
        if self.BrillouinScan.Cancel_Flag == False:
            currExp = self.session.experimentList[self.BrillouinScan.saveExpIndex]
            currScanIdx = self.session.experimentList[self.BrillouinScan.saveExpIndex].size() - 1
            SD = self.BrillouinScan.SDcal
            FSR = self.BrillouinScan.FSRcal
            if ~np.isnan(SD):
                self.allParameters.child('More Settings').child('SD').setValue(SD)
            if ~np.isnan(FSR):
                self.allParameters.child('More Settings').child('FSR').setValue(FSR)
        self.BrillouinScan.motorPosUpdateSig.disconnect(self.MotorPositionUpdate2)
        #self.hardwareGetTimer.start(60000)
        self.motorPositionTimer.start(500)
        self.BrillouinScan.Cancel_Flag = False
        print('Scan completed')

    def toggleReference(self, sliderParam, state):
        # print("[toggleReference]")
        # state == True --> Reference
        # state == False --> Sample
        if state:
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorDeviceThread.setRefState(True)
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value())
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorDeviceThread.setRefState(False)
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    def toggleFluorescence(self, sliderParam, state):
        #print("[toggleFluorescence]")
        # state == True --> Fluorescence
        # state == False --> Brightfield
        if state:
            # Switch to filter index 2:
            self.ZaberDevice.moveFilter(2)
            # Turn off Brightfield lamp, turn on fluorescence lamps
            self.ZaberDevice.lightSwitch('white', True)
            self.ZaberDevice.lightSwitch('blue', True)
            self.ZaberDevice.lightSwitch('red', True)
            self.ZaberDevice.lightSwitch('trans', False)
            # Change Mako camera exposure time
            self.MakoDeviceThread.setExpTime(self.allParameters.child('Microscope Camera').child('Fluoresc. Exp.').value())
        else:
            # Switch to filter index 1:
            self.ZaberDevice.moveFilter(1)
            # Turn on Brightfield lamp, turn off fluorescence lamps
            self.ZaberDevice.lightSwitch('white', False)
            self.ZaberDevice.lightSwitch('blue', False)
            self.ZaberDevice.lightSwitch('red', False)
            self.ZaberDevice.lightSwitch('trans', True)
            # Change Mako camera exposure time
            self.MakoDeviceThread.setExpTime(self.allParameters.child('Microscope Camera').child('Brightfield Exp.').value())

    def bgSubtraction(self, sliderParam, state):
        if state:
            self.ShutterDevice.closeBGshutter()
            self.AndorDeviceThread.startBGsubtraction()
            #print("Spectrometer background subtraction ON")
            # Wait until BG subtraction is complete before re-opening shutter
            while self.AndorDeviceThread.checkBGsubtraction():
                time.sleep(0.1)
            self.ShutterDevice.openBGshutter()
        else:
            self.AndorDeviceThread.stopBGsubtraction()
            #print("Spectrometer background subtraction OFF")

    #def switchAutoExp(self, sliderParam, state):
    #    if state:
    #        self.AndorDeviceThread.setAutoExp(True)
    #        print("Spectrometer auto exposure ON")
    #    else:
    #        self.AndorDeviceThread.setAutoExp(False)
    #        self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())
    #        print("Spectrometer auto exposure OFF")
    #        # self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    #############################################################################################
    # This next group of methods callback methods to display acquired data #
    #############################################################################################
    
    def UpdateAmbientTemp(self, temperature):
        # print('UpdateAmbientTemp')
        self.allParameters.child('More Settings').child('Ambient Temp.').setValue(temperature)

    # updates the figure containing the Brillouin sequence. newData is a list
    def UpdateSampleBrillouinSeqPlot(self, interPeakDist):
        # print("[UpdateBrillouinSeqPlot]")
        SD = self.allParameters.child('More Settings').child('SD').value()
        FSR = self.allParameters.child('More Settings').child('FSR').value()
        if ~np.isnan(interPeakDist):
            newData = [0.5*(FSR - SD*interPeakDist)]
            newData2 = newData
        else:
            newData = [np.nan]
            newData2 = [np.nan]

        # Update heatmap with Brillouin data
        if len(newData)+len(self.heatmapScanData) > self.maxRowPoints*self.maxColPoints:
            self.heatmapScanData = np.array([])
        self.heatmapScanData = np.append(self.heatmapScanData, newData)
        heatmapSeq = np.pad(self.heatmapScanData, (0, self.maxRowPoints*self.maxColPoints - self.heatmapScanData.shape[0]), 'constant')
        heatmapSeq[np.isnan(heatmapSeq)] = 0 #Remove NaNs
        heatmapArr = np.reshape(heatmapSeq, (-1, self.maxRowPoints))   # self.maxRowPoints columns
        #heatmapArr[1::2, :-self.calPoints] = np.fliplr(heatmapArr[1::2, :-self.calPoints]) # Match S-shaped line scan pattern
        heatmapArr = np.rot90(heatmapArr, 1, (1,0)) # Rotate to match XY axes
        colormapLow = self.allParameters.child('Display').child('Colormap (min.)').value()
        colormapHigh = self.allParameters.child('Display').child('Colormap (max.)').value()
        self.heatmapImage.setImage(heatmapArr, levels=(colormapLow, colormapHigh))

        # Update Brillouin vs. time plot
        if len(newData)+len(self.sampleScanDepthData) > self.maxScanPoints:
            if self.calPoints > 0:
                self.sampleScanDepthData = np.array(newData)
            else:
                t = self.maxScanPoints - len(newData) - len(self.sampleScanDepthData)
                self.sampleScanDepthData = np.roll(self.sampleScanDepthData, t)
                self.sampleScanDepthData[self.maxScanPoints - len(newData):] = newData
        else:
            self.sampleScanDepthData = np.append(self.sampleScanDepthData, newData)

        xdata = np.arange(len(self.sampleScanDepthData))
        xdata = xdata[~np.isnan(self.sampleScanDepthData)]
        ydata = self.sampleScanDepthData[~np.isnan(self.sampleScanDepthData)]
        self.sampleScanDepthItem.setData(x=xdata, y=ydata)

    # Plot fitted Brillouin spectrum
    # curvedata is a tuple of (raw spectrum, fitted spectrum)
    def UpdateSampleSpectrum(self, curveData):
        # print("[UpdateSpectrum]")
        rawSpect = curveData[0]
        fitSpect = curveData[1]

        xdata = np.arange(len(rawSpect))

        self.sampleSpectrumItem.setData(x=xdata, y=np.array(rawSpect))
        if len(fitSpect[~np.isnan(fitSpect)]) == len(rawSpect):
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.array(fitSpect))
        else:
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.zeros(len(rawSpect)))

        if self.sampleSpecSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
            print('[UpdateSpectrum] Resizing spectrograph')
            self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
            self.sampleSpecSeriesSize = 0

        #print('self.sampleSpecSeriesSize=', self.sampleSpecSeriesSize)
        #print('self.maxScanPoints=', self.maxScanPoints)
        if self.sampleSpecSeriesSize < self.maxScanPoints:
            self.sampleSpecSeriesData[self.sampleSpecSeriesSize,:] = rawSpect
            self.sampleSpecSeriesSize += 1
        else:
            if self.calPoints > 0:
                self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
                self.sampleSpecSeriesData[0,:] = rawSpect
                self.sampleSpecSeriesSize = 1
            else:
                self.sampleSpecSeriesData = np.roll(self.sampleSpecSeriesData, -1, axis=0)
                self.sampleSpecSeriesData[-1, :] = rawSpect

        maximum = self.sampleSpecSeriesData.max()
        sampleSpecSeriesDataScaled = self.sampleSpecSeriesData #* (255.0 / maximum)
        self.sampleSpecSeriesImage.setImage(sampleSpecSeriesDataScaled)

    # Update raw Andor image
    def AndorSampleProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        self.SampleImage.setImage(image.transpose((1,0)))

    # Update binned Andor image
    def AndorBinnedProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        self.BinnedImage.setImage(image.transpose((1,0)))

    # Update the CMOS camera image
    def MakoProcessUpdate(self, makoData):
        #print("[MakoProcessUpdate]")
        image = makoData
        self.CMOSImage.setImage(image)

    def createNewSession(self):
        dialog = QtGui.QFileDialog()
        options = QtGui.QFileDialog.Options()
        options |= QtGui.QFileDialog.DontUseNativeDialog
        dialog.setOptions(options)
        dialog.setDefaultSuffix('hdf5')
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setNameFilter("HDF5 Files(*.hdf5)")
        if dialog.exec_():
            filename = str(dialog.selectedFiles()[0])
        else:
            return

        self.dataFileName = filename
        self.sessionName.setText(filename)

        #create a single new session
        self.session = SessionData(ntpath.basename(self.dataFileName), filename=filename)

        # create tree model (for display)
        self.model = BrillouinTreeModel()
        self.treeView.setModel(self.model)
        self.treeView.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self.createTreeMenu)
        self.treeView.selectionModel().selectionChanged.connect(self.treeSelectionChanged)

        self.model.session = self.session

        self.session.updateTreeViewSig.connect(
            lambda updateIndices:self.model.updateTree(updateIndices, self.treeView))
        self.session.addNewExperiment()

        self.treeView.setEnabled(True)


    def closeEvent(self,event):
        print("Program Shutdown")
        if self.dataFile:
            self.dataFile.close()

        #self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()
        
        # Turn all lamps off:
        self.ZaberDevice.lightSwitch('white', False)
        self.ZaberDevice.lightSwitch('blue', False)
        self.ZaberDevice.lightSwitch('red', False)
        self.ZaberDevice.lightSwitch('trans', False)

        self.stop_event.set()
        while self.AndorDeviceThread.isRunning():
            time.sleep(0.1)
        while self.MakoDeviceThread.isRunning():
            time.sleep(0.1)

        self.MakoDeviceThread.shutdown()
        self.TempSensorDeviceThread.shutdown()
        self.ZaberDevice.shutdown()
        self.ShutterDevice.shutdown()
        self.SynthDevice.shutdown()

        event.accept() #closes the application

    #############################################################################################
    # This next group of methods are for experiment/data management (treeView)                  #
    #############################################################################################
    #TODO: save to file when renaming

    def createTreeMenu(self, position):
        menu = QtGui.QMenu()
        modelIndex = self.treeView.indexAt(position)
        item = self.model.itemFromIndex(modelIndex)

        if item is not None:
            lvl = 0
            k = item.parent()
            while k is not None:
                k = k.parent()
                lvl += 1

            if lvl == 0:
                setActiveAction = menu.addAction("Set as Active")
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == setActiveAction:
                    self.model.setActiveExperiment(modelIndex.row())
            elif lvl == 1:
                if (item.parent().child(item.row(), 0).data(role=UserItemRoles.IsDeletedRole).toBool() == False):
                    deleteText = "Delete Scan"
                else:
                    deleteText = "Undelete Scan"
                deleteAction = menu.addAction(deleteText)
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == deleteAction:
                    self.model.deleteScan(item)
        else:
            newAction = menu.addAction("New Experiment")
            action = menu.exec_(self.treeView.mapToGlobal(position))
            if action == newAction:
                self.session.addNewExperiment()


    def treeSelectionChanged(self, selected, deselected):
        #print('selected =', selected)
        #print('selected.indexes() =', selected.indexes())
        idx = selected.indexes()[0]
        item = self.model.itemFromIndex(idx)
        if item.parent() is None:
            # print("Experiment selected")
            return
         
    def mainUI(self):
        self.newSessionBtn.clicked.connect(self.createNewSession)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI = App()
    GUI.show()
    sys.exit(app.exec_())
