from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal

from ExperimentData import *

# Only a small number of fields are editable from the treeView interface:
# Experiment name
# Scan notes
# Scan "deleted"
# Experiment "deleted"

class UserItemRoles:
	IsDeletedRole = QtCore.Qt.UserRole + 1

class BrillouinTreeModel(QtGui.QStandardItemModel):
	activeExperimentSig = pyqtSignal(int)	# this signal will trigger heatmap update

	def __init__(self, parent=None):
		super(BrillouinTreeModel, self).__init__(parent)

		self.activeExperiment = None
		self.setHorizontalHeaderLabels(['Name', 'Time', 'Note'])

		self.session = None
		self.itemChanged.connect(self.updateSession)

	def updateSession(self, changedItem):
		# list all possibilities 

		if changedItem.parent() is None:	# experiment changed
			expIdx = changedItem.row()
			expData = self.session.experimentList[expIdx]
			if (str(changedItem.text()) != expData.name):		# Change Experiment name
				expData.name = str(changedItem.text())
				expNameField = 'Exp_' + str(expIdx) + '/name'
				self.session.saveToFile(updateFieldOnly=True, fieldPath=expNameField)

		else:								# scan changed
			scanIdx = changedItem.row()
			headItem = changedItem.parent().child(scanIdx, 0)
			noteItem = changedItem.parent().child(scanIdx, 2)
			scanData = self.session.experimentList[changedItem.parent().row()].scanList[scanIdx]


			if (str(noteItem.text()) != scanData.note):			# Change scan note
				scanData.note = str(noteItem.text())
				noteField = 'Exp_' + str(changedItem.parent().row()) + '/Scan_' + str(scanIdx) + '/note'
				self.session.saveToFile(updateFieldOnly=True, fieldPath=noteField)

			#if (headItem.data(role=UserItemRoles.IsDeletedRole).toBool() != scanData.deleted):	# change deleted flag
			if (headItem.data(role=UserItemRoles.IsDeletedRole) != scanData.deleted):	# change deleted flag
				#scanData.deleted = headItem.data(role=UserItemRoles.IsDeletedRole).toBool()
				scanData.deleted = headItem.data(role=UserItemRoles.IsDeletedRole)
				deletedField = 'Exp_' + str(changedItem.parent().row()) + '/Scan_' + str(scanIdx) + '/deleted'
				self.session.saveToFile(updateFieldOnly=True, fieldPath=deletedField)
				self.activeExperimentSig.emit(self.activeExperiment)

	def deleteScan(self, item):
		# Set foreground color to red
		itemRow = item.row()
		parent = item.parent()
		item1 = parent.child(itemRow, 0)
		if (item1.data(role=UserItemRoles.IsDeletedRole).toBool() == False):
			# delete
			for k in range(parent.columnCount()):
			    parent.child(itemRow, k).setForeground(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
			item1.setData(True, role=UserItemRoles.IsDeletedRole)
		else:
			# undelete
			for k in range(parent.columnCount()):
			    parent.child(itemRow, k).setForeground(QtGui.QBrush(QtGui.QColor(0, 0, 0)))
			item1.setData(False, role=UserItemRoles.IsDeletedRole)


	def setActiveExperiment(self, expIdx):
		if (self.activeExperiment == expIdx):
		    return
		if self.activeExperiment is not None:
			oldActiveItem = self.item(self.activeExperiment)
			oldActiveItem.setForeground(QtGui.QBrush(QtGui.QColor(0, 0, 0)))
			oldActiveItem.setBackground(QtGui.QBrush(QtGui.QColor(255, 255, 255)))

		self.activeExperiment = expIdx
		expItem = self.item(expIdx)
		expItem.setForeground(QtGui.QBrush(QtGui.QColor(0, 255, 0)))
		expItem.setBackground(QtGui.QBrush(QtGui.QColor(0, 0, 0)))

		self.activeExperimentSig.emit(expIdx)



	# updates self.treeView to match data in self.session
	# arguments corresponds to those in ScanManager.saveToFile
	def updateTree(self, updateIndices, treeView):
		# print("[updateTree]")
		# if fieldOnly:
		#     #TODO: implement this
		#     return

		for updateIdx in updateIndices:
			expIdx = updateIdx[0]
			if (expIdx >= self.rowCount()):	# create new experiment
				newExpItem = QtGui.QStandardItem(self.session.experimentList[expIdx].name)
				self.appendRow(newExpItem)
				# span container columns
				treeView.setFirstColumnSpanned(self.rowCount()-1, treeView.rootIndex(), True)
				# self.treeView.setCurrentIndex(parent.index())
				treeView.selectionModel().select(
					newExpItem.index(), QtCore.QItemSelectionModel.ClearAndSelect|QtCore.QItemSelectionModel.Rows)
				self.setActiveExperiment(expIdx)

			# Navigate to this index in the treeView
			expItem = self.item(expIdx)
			treeView.expand(expItem.index())

			scanIndices = updateIdx[1]
			if scanIndices == []:
				scanIndices = range(self.session.experimentList[expIdx].size())
			for scanIdx in scanIndices:
				# Check to see if scan already exist in tree
				scan = self.session.experimentList[expIdx].scanList[scanIdx]
				if scanIdx >= expItem.rowCount():
					# create new item 
					child1 = QtGui.QStandardItem('{}'.format(scanIdx))
					child1.setFlags(child1.flags() ^ ~QtCore.Qt.ItemIsEditable)
					child1.setForeground(QtGui.QBrush(QtGui.QColor(0, 0, 0)))
					child1.setData(False, role=UserItemRoles.IsDeletedRole)

					child2 = QtGui.QStandardItem(scan.timestamp)
					child2.setFlags(child2.flags() ^ ~QtCore.Qt.ItemIsEditable)
					child2.setForeground(QtGui.QBrush(QtGui.QColor(0, 0, 0)))

					child3 = QtGui.QStandardItem(scan.note)
					expItem.appendRow([child1, child2, child3])

					# highlight the last inserted scan
					newScanTreeIndex = expItem.child(scanIdx,1).index()
					treeView.selectionModel().select(
						newScanTreeIndex, QtCore.QItemSelectionModel.ClearAndSelect|QtCore.QItemSelectionModel.Rows)
				else:
					pass
					# TODO: update existing item
					# scanItem = 

	# def treeSelectionChanged(self, selected, deselected):
	# 	idx = selected.indexes()[0]
	# 	item = self.itemFromIndex(idx)
	# 	if item.parent() is None:
	# 	    print("Experiment selected")
	# 	    return
	# 	else:
	# 	    expIdx = item.parent().row()
	# 	    scanIdx = item.row()
	# 	    print("Exp_%d Scan_%d selected" % (expIdx, scanIdx))

		    # TODO: emit signal to update heatmap
		    # if len(self.heatmapScatterLastClicked)==1:
		    #     p = self.heatmapScatterLastClicked[0]
		    #     if p.data()[0] == expIdx and p.data()[1] == scanIdx:
		    #         return

		    # for p in self.heatmapScatterLastClicked:
		    #     p.resetPen()
		    # p = None
		    # for p1 in self.heatmapScatter.points():
		    #     if p1.data()[0] == expIdx and p1.data()[1] == scanIdx:
		    #         p = p1
		    #         continue
		    # if p is None:   # point not found. Should never occur
		    #     return

		    # p.setPen('w', width=2)
		    # self.heatmapScatterLastClicked = [p]    # only allow single selection   


