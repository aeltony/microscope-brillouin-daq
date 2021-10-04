from datetime import datetime
import numpy as np
import h5py
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

# This file describe classes that contain all the experimental data
# Data is save in a hierachy of Session (patient) --> Experiment (i.e. left and right eyes) --> Scans(i.e. individual A-lines)
# A Session can have it's own data, in addition to multiple Experiments. Similarly,
# an Experiment can have it's own data, in addition to multiple Scans

# Each class contain functions that save the content in HDF5 format. 
# The format of the save hdf5 data is a hierachy of root(session)/Experiment/Scan
# where each is a subgroup of the data file. Metadata is directly attached to the subgroups
# Only SessionData.saveToFile() should be called, which is responsible for opening and 
# closing the desired file. It is the user's responsibility to ensure that the data 
# in these classes is consistent with the data in the file.

# Most data should not be changed once saved to file. One exception is "note", which is a string
# attribute in all of the classes below. Therefore we also keep track of which data has
# been modified since last saved


#TODO: save backup files to avoid data corruption
#TOD: auto save to file when modifying to prevent potential out of sync between data struct and file

# pList is a list of dictionaries
def generateParameterList(pTreeParams, pTree, parentName = ''):
	pList = []
	for item in pTreeParams:
		if item['type'] == 'group':
			parentPath = item['name'] + '/'
			subTree = pTree.child(item['name'])
			pList += (generateParameterList(item['children'], subTree, parentPath))

		elif item['type'] == 'action' or item['type'] == 'action2':
			pass

		else:
			# all numerical types here
			pList.append( (parentName + item['name'], pTree.child(item['name']).value()) )
	return pList



class SessionData(QtCore.QObject):
	updateTreeViewSig = pyqtSignal('PyQt_PyObject')

	# Attributes to save
	# TODO: set the types of these attributes 
	savedAttributes = ['name', 'timestamp', 'note']	# these are hdf5 attributes
	savedDataset = []

	def __init__(self, name, timestamp = 'None', filename = None):
		super(SessionData,self).__init__()
		self.filename = filename
		self.name = name
		self.timestamp = timestamp
		self.experimentList = []
		self.note = ""
		#self.updateTreeViewSig.emit([])

	def size(self):
		return len(self.experimentList)

	def addExperiment(self, experiment):
		# set index and parent
		experiment.index = self.size()
		experiment.parent = self
		self.experimentList.append(experiment)

	def addNewExperiment(self, filename = None):
		newExp = ExperimentData("New Experiment")
		self.addExperiment(newExp)
		if filename is not None:
			self.saveToFile([(newExp.index,[])], filename = filename )
		elif self.filename is not None:
			self.saveToFile( [(newExp.index,[])] )

	def getNote(self):
		return self.note

	def addNote(self, note):
		self.note = note

	# expScanIndex is a list of tuples [(exp1, [scan1, scan2, ...scanN]), (exp3, [scan2, scan3])]
	# if expScanIndex is empty list, the save all exp
	# if the second element of a tuple is the empty list, then save all scans
	# updateSelf when saving for the first time
	# This method can also save a single field, which is useful such as when a text note is edited
	# "field" is a string "(attribute) / experiment index or attribute / scan index or attribute" such as
	# "Exp_0/temperature", or "name", or "Exp_1/Scan_2/deleted"
	# TODO: some things should be able to change, such as index, but we current do not enforce that
	def saveToFile(self, expScanIndices = [], filenameIn=None, updateFieldOnly = False, fieldPath = None, updateSelf=True):
		if filenameIn is not None:
			filename = filenameIn
		elif self.filename is not None:
			filename = self.filename
		else:
			print("[SessionData/saveToFile] filename not provided")
			return
		#print("[SessionData/saveToFile] Session File Save: " + filename)
		with h5py.File(filename, 'a') as fHandle:
			if updateFieldOnly:	#TODO: throw errors when the field path is invalid
				pathElements = fieldPath.split('/')
				if pathElements[0][0:4]=='Exp_':
					exp = self.experimentList[int(pathElements[0][4:])]
					if pathElements[1][0:5]=='Scan_':
						scan = exp.scanList[int(pathElements[1][5:])]
						attribute = pathElements[2]
						fHandle[pathElements[0]+'/'+pathElements[1]].attrs[attribute] = scan.__dict__[attribute]
					else:
						# modifying an Experiment attribute
						attribute = pathElements[1]
						fHandle[pathElements[0]].attrs[attribute] = exp.__dict__[attribute]
				else:
					# modifying a Session attribute
					attribute = pathElements[0]
					fHandle.attrs[attribute] = self.__dict__[attribute]
				return

			if updateSelf:
				pass	#TODO: save attr here
			if expScanIndices==[]:
				for exp in self.experimentList:
					exp.saveToFile(fHandle, [])
			else:
				for expIndex, scanListIndices in expScanIndices:
					self.experimentList[expIndex].saveToFile(fHandle, scanListIndices)

			for data in SessionData.savedDataset:
				if hasattr(self, data):
					# first check if file already has this dataset
					datasetName = data
					if datasetName in fHandle:	# delete if already exist
						del fHandle[datasetName]
					fHandle.create_dataset(datasetName, data=self.__dict__[data])

			for attribute in SessionData.savedAttributes:
				if hasattr(self, attribute):
					fHandle.attrs[attribute] = self.__dict__[attribute]

		self.updateTreeViewSig.emit(expScanIndices)



# parent is the containing Session
class ExperimentData:
	# Attributes to save
	# TODO: set the types of these attributes 
	savedAttributes = ['timestamp', 'note', 'deleted']	# these are hdf5 attributes
	savedDataset = []							# these are hdf5 dataset

	def __init__(self, name, timestamp = 'None', parent = None):
		self.name = name
		self.index = 0
		self.timestamp = timestamp
		self.scanList = []
		self.note = ""
		self.parent = parent
		self.deleted = False		# data is never completely deleted, only flagged

	# get indices of not deleted scans
	#def getActiveScanIndices(self):
	#	scanIndexList = []
	#	idx = 0
	#	for scan in self.scanList:
	#		if scan.deleted:
	#			idx += 1
	#			continue
	#		scanIndexList.append(idx)
	#		idx += 1
	#	return scanIndexList

	def size(self):
		return len(self.scanList)

	def addScan(self, scan):
		# increment index and set parent
		scan.index = self.size()
		scan.parent = self
		self.scanList.append(scan)

	def getNote(self):
		return self.note

	def addNote(self, note):
		self.note = note

	def getGroupName(self):
		return 'Exp_' + str(self.index)	

	# def generateTestData(self):
	# 	self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
	# 	for k in range(10):
	# 		fakeScan = ScanData()
	# 		fakeScan.generateTestData(k)
	# 		self.addScan(fakeScan)
	# 	self.note = "Exp" + self.name + "notes blah"

	def saveToFile(self, fHandle, scanList, updateSelf=True):
		if updateSelf:
			pass #TODO: save attr here
		if scanList == []:
			scanList = range(len(self.scanList))
		for scanIndex in scanList:
			self.scanList[scanIndex].saveToFile(fHandle)

		datasetPath = self.getGroupName() + '/'
		if not datasetPath in fHandle:
			fHandle.create_group(self.getGroupName())

		for data in ExperimentData.savedDataset:
			if hasattr(self, data):
				# first check if file already has this dataset
				datasetName = datasetPath + data
				if datasetName in fHandle:	# delete if already exist
					del fHandle[datasetName]
				fHandle.create_dataset(datasetName, data=self.__dict__[data])

		gp = fHandle[datasetPath]
		for attribute in ExperimentData.savedAttributes:
			if hasattr(self, attribute):
				gp.attrs[attribute] = self.__dict__[attribute]



class ScanData:
	# if ScanData has attribute flattenedParamList, all of the parameters in there will be saved as attributes

	# Attributes to save
	# TODO: set the types of these attributes
	savedAttributes = ['timestamp', 'note', 'deleted']	# these are hdf5 attributes

	# these are hdf5 dataset
	# Ensure these variables are assigned
	savedDataset = ['ScanData',
				 'CalFreq',
				 'TempList',
				 'AndorImage',
				 'BrightfieldImage',
				 'FluorescenceImage',
				 'SpecList',
				 'MotorCoords',
				 'SD',
				 'FSR']

	# scanAttribuets and scanSettings are dictionaries
	# parent is the containing Experiment
	def __init__(self, scanAttributes = None, scanSettings = None, timestamp = 'None', parent = None):
		self.index = 0			# unique index for the scan
		self.scanAttributes = scanAttributes
		self.scanSettings = scanSettings
		self.timestamp = timestamp
		self.note = ""
		self.parent = parent
		self.deleted = False
		self.temperature = -273

	def getNote(self):
		return self.note

	def addNote(self, note):
		self.note = note

	def saveData(self, filepath):
		return 0

	# load data from files
	def loadData(self):
		return 0

	def saveToFile(self, fHandle):
		datasetPath = self.parent.getGroupName() + '/Scan_' + str(self.index) + '/'
		if not datasetPath in fHandle:
			fHandle.create_group(datasetPath)

		for data in ScanData.savedDataset:
			if hasattr(self, data):
				# first check if file already has this dataset
				datasetName = datasetPath + data
				if datasetName in fHandle:	# delete if already exist
					pass
					#del fHandle[datasetName]
				if hasattr(self.__dict__[data], '__len__') and (not isinstance(self.__dict__[data], str)):
					#print('Compressing data:' + datasetName)
					fHandle.create_dataset(datasetName, data=self.__dict__[data], chunks=True, \
						shuffle=True, compression='lzf')
					# Once saved to file, delete data from working memory
					try:
						self.__dict__[data] = None
					except:
						print('[ExperimentData/saveToFile] Could not free memory (1)')
				else:
					# print('Not compressing data:' + datasetName)
					fHandle.create_dataset(datasetName, data=self.__dict__[data])
					# Once saved to file, delete data from working memory
					try:
						self.__dict__[data] = None
					except:
						print('[ExperimentData/saveToFile] Could not free memory (2)')

		gp = fHandle[datasetPath]
		for attribute in ScanData.savedAttributes:
			if hasattr(self, attribute):
				gp.attrs[attribute] = self.__dict__[attribute]

		if hasattr(self, 'flattenedParamList'):
			for item in self.flattenedParamList:
				if item[1] != None:
					attrName = 'paramtree/' + item[0]
					gp.attrs[attrName] = item[1]
			# Once saved to file, delete data from working memory
			try:
				self.__dict__['flattenedParamList'] = None
			except:
				print('[ExperimentData/saveToFile] Could not free memory (3)')



	# def generateTestData(self, index, fakeArray = None):
	# 	self.index = index
	# 	self.scanAttributes = {'Start':-200, 'End':1200}
	# 	self.scanSettings = {'Exposure':0.2, 'other':'what'}
	# 	self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")