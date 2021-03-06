# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'qt_ui_simplified.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1900, 1000)
        MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.treeView = QtWidgets.QTreeView(self.centralwidget)
        self.treeView.setGeometry(QtCore.QRect(280, 10, 300, 960))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(2)
        sizePolicy.setHeightForWidth(self.treeView.sizePolicy().hasHeightForWidth())
        self.treeView.setSizePolicy(sizePolicy)
        self.treeView.setMinimumSize(QtCore.QSize(300, 0))
        self.treeView.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeView.setObjectName("treeView")
        self.parameterTreeWidget = ParameterTree(self.centralwidget)
        self.parameterTreeWidget.setGeometry(QtCore.QRect(10, 70, 260, 900))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(4)
        sizePolicy.setHeightForWidth(self.parameterTreeWidget.sizePolicy().hasHeightForWidth())
        self.parameterTreeWidget.setSizePolicy(sizePolicy)
        self.parameterTreeWidget.setMinimumSize(QtCore.QSize(260, 0))
        self.parameterTreeWidget.setAnimated(True)
        self.parameterTreeWidget.setHeaderHidden(True)
        self.parameterTreeWidget.setObjectName("parameterTreeWidget")
        self.parameterTreeWidget.headerItem().setText(0, "1")
        self.sessionName = QtWidgets.QLineEdit(self.centralwidget)
        self.sessionName.setEnabled(True)
        self.sessionName.setGeometry(QtCore.QRect(10, 34, 165, 22))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.sessionName.setFont(font)
        self.sessionName.setText("")
        self.sessionName.setAlignment(QtCore.Qt.AlignCenter)
        self.sessionName.setReadOnly(True)
        self.sessionName.setObjectName("sessionName")
        self.session_label = QtWidgets.QLabel(self.centralwidget)
        self.session_label.setGeometry(QtCore.QRect(10, 10, 121, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.session_label.setFont(font)
        self.session_label.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.session_label.setObjectName("session_label")
        self.newSessionBtn = QtWidgets.QPushButton(self.centralwidget)
        self.newSessionBtn.setGeometry(QtCore.QRect(183, 32, 85, 26))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.newSessionBtn.setFont(font)
        self.newSessionBtn.setObjectName("newSessionBtn")
        self.graphicsViewCMOS = GraphicsView(self.centralwidget)
        self.graphicsViewCMOS.setGeometry(QtCore.QRect(590, 45, 440, 440))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.graphicsViewCMOS.sizePolicy().hasHeightForWidth())
        self.graphicsViewCMOS.setSizePolicy(sizePolicy)
        self.graphicsViewCMOS.setMinimumSize(QtCore.QSize(440, 440))
        self.graphicsViewCMOS.setObjectName("graphicsViewCMOS")
        self.graphicsViewSampleSpecSeries = GraphicsView(self.centralwidget)
        self.graphicsViewSampleSpecSeries.setGeometry(QtCore.QRect(1175, 570, 715, 400))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(3)
        sizePolicy.setHeightForWidth(self.graphicsViewSampleSpecSeries.sizePolicy().hasHeightForWidth())
        self.graphicsViewSampleSpecSeries.setSizePolicy(sizePolicy)
        self.graphicsViewSampleSpecSeries.setMinimumSize(QtCore.QSize(0, 0))
        self.graphicsViewSampleSpecSeries.setObjectName("graphicsViewSampleSpecSeries")
        self.graphicsViewBinned = GraphicsView(self.centralwidget)
        self.graphicsViewBinned.setGeometry(QtCore.QRect(1060, 530, 105, 440))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.graphicsViewBinned.sizePolicy().hasHeightForWidth())
        self.graphicsViewBinned.setSizePolicy(sizePolicy)
        self.graphicsViewBinned.setMinimumSize(QtCore.QSize(105, 420))
        self.graphicsViewBinned.setObjectName("graphicsViewBinned")
        self.graphicsViewSampleScanDepth = GraphicsView(self.centralwidget)
        self.graphicsViewSampleScanDepth.setGeometry(QtCore.QRect(1450, 220, 440, 340))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(3)
        sizePolicy.setHeightForWidth(self.graphicsViewSampleScanDepth.sizePolicy().hasHeightForWidth())
        self.graphicsViewSampleScanDepth.setSizePolicy(sizePolicy)
        self.graphicsViewSampleScanDepth.setMinimumSize(QtCore.QSize(440, 340))
        self.graphicsViewSampleScanDepth.setObjectName("graphicsViewSampleScanDepth")
        self.graphicsViewHeatmap = GraphicsView(self.centralwidget)
        self.graphicsViewHeatmap.setGeometry(QtCore.QRect(590, 530, 460, 440))
        self.graphicsViewHeatmap.setMinimumSize(QtCore.QSize(0, 0))
        self.graphicsViewHeatmap.setObjectName("graphicsViewHeatmap")
        self.graphicsViewSample = GraphicsView(self.centralwidget)
        self.graphicsViewSample.setGeometry(QtCore.QRect(1040, 10, 850, 200))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.graphicsViewSample.sizePolicy().hasHeightForWidth())
        self.graphicsViewSample.setSizePolicy(sizePolicy)
        self.graphicsViewSample.setMinimumSize(QtCore.QSize(850, 200))
        self.graphicsViewSample.setObjectName("graphicsViewSample")
        self.graphicsViewSampleSpectrum = GraphicsView(self.centralwidget)
        self.graphicsViewSampleSpectrum.setGeometry(QtCore.QRect(1040, 220, 400, 300))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(2)
        sizePolicy.setHeightForWidth(self.graphicsViewSampleSpectrum.sizePolicy().hasHeightForWidth())
        self.graphicsViewSampleSpectrum.setSizePolicy(sizePolicy)
        self.graphicsViewSampleSpectrum.setMinimumSize(QtCore.QSize(400, 300))
        self.graphicsViewSampleSpectrum.setObjectName("graphicsViewSampleSpectrum")
        self.session_label_8 = QtWidgets.QLabel(self.centralwidget)
        self.session_label_8.setGeometry(QtCore.QRect(1180, 540, 260, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.session_label_8.setFont(font)
        self.session_label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.session_label_8.setObjectName("session_label_8")
        self.session_label_9 = QtWidgets.QLabel(self.centralwidget)
        self.session_label_9.setGeometry(QtCore.QRect(590, 500, 440, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.session_label_9.setFont(font)
        self.session_label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.session_label_9.setObjectName("session_label_9")
        self.session_label_10 = QtWidgets.QLabel(self.centralwidget)
        self.session_label_10.setGeometry(QtCore.QRect(590, 15, 440, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.session_label_10.setFont(font)
        self.session_label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.session_label_10.setObjectName("session_label_10")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Brillouin Scanning Interface"))
        self.session_label.setText(_translate("MainWindow", "Session Name"))
        self.newSessionBtn.setText(_translate("MainWindow", "New Session"))
        self.session_label_8.setText(_translate("MainWindow", "Spectrograph"))
        self.session_label_9.setText(_translate("MainWindow", "Brillouin Map"))
        self.session_label_10.setText(_translate("MainWindow", "Live View"))

from pyqtgraph.parametertree import ParameterTree
from pyqtgraph.widgets.GraphicsView import GraphicsView

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

