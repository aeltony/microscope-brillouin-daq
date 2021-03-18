import pyqtgraph as pg
from PyQt5 import QtGui, QtCore
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType



class ActionParameterItem2(ParameterItem):
    def __init__(self, param, depth):
        ParameterItem.__init__(self, param, depth)
        self.layoutWidget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout()
        self.layoutWidget.setLayout(self.layout)
        bt = param.opts['ButtonText']
        self.button = QtGui.QPushButton(bt[0])
        self.button2 = QtGui.QPushButton(bt[1])
        #self.layout.addSpacing(100)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.button2)
        self.layout.addStretch()
        self.layout.setContentsMargins(2,2,2,2)
        self.button.clicked.connect(self.buttonClicked)
        self.button2.clicked.connect(self.buttonClicked2)
        param.sigNameChanged.connect(self.paramRenamed)
        self.setText(0, '')
        
    def treeWidgetChanged(self):
        ParameterItem.treeWidgetChanged(self)
        tree = self.treeWidget()
        if tree is None:
            return
        
        tree.setFirstItemColumnSpanned(self, True)
        tree.setItemWidget(self, 0, self.layoutWidget)
        
    def paramRenamed(self, param, name):
        self.button.setText(name)
        
    def buttonClicked(self):
        self.param.activate()
        
    def buttonClicked2(self):
        self.param.activate2()

class ActionParameterItem3(ParameterItem):
    def __init__(self, param, depth):
        ParameterItem.__init__(self, param, depth)
        self.layoutWidget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout()
        self.layoutWidget.setLayout(self.layout)
        bt = param.opts['ButtonText']
        self.button = QtGui.QPushButton(bt[0])
        self.button2 = QtGui.QPushButton(bt[1])
        self.button3 = QtGui.QPushButton(bt[2])
        #self.layout.addSpacing(100)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.button2)
        self.layout.addWidget(self.button3)
        self.layout.addStretch()
        self.layout.setContentsMargins(2,2,2,2)
        self.button.clicked.connect(self.buttonClicked)
        self.button2.clicked.connect(self.buttonClicked2)
        self.button3.clicked.connect(self.buttonClicked3)
        param.sigNameChanged.connect(self.paramRenamed)
        self.setText(0, '')
        
    def treeWidgetChanged(self):
        ParameterItem.treeWidgetChanged(self)
        tree = self.treeWidget()
        if tree is None:
            return
        
        tree.setFirstItemColumnSpanned(self, True)
        tree.setItemWidget(self, 0, self.layoutWidget)
        
    def paramRenamed(self, param, name):
        self.button.setText(name)
        
    def buttonClicked(self):
        self.param.activate()
        
    def buttonClicked2(self):
        self.param.activate2()

    def buttonClicked3(self):
        self.param.activate3()

class ActionParameter2(Parameter):
    """Used for displaying a button within the tree."""
    itemClass = ActionParameterItem2
    sigActivated = QtCore.Signal(object)
    sigActivated2 = QtCore.Signal(object)
    
    def activate(self):
        self.sigActivated.emit(self)
        self.emitStateChanged('activated', None)

    def activate2(self):
        self.sigActivated2.emit(self)
        self.emitStateChanged('activated', None)

class ActionParameter3(Parameter):
    """Used for displaying a button within the tree."""
    itemClass = ActionParameterItem3
    sigActivated = QtCore.Signal(object)
    sigActivated2 = QtCore.Signal(object)
    sigActivated3 = QtCore.Signal(object)
    
    def activate(self):
        self.sigActivated.emit(self)
        self.emitStateChanged('activated', None)

    def activate2(self):
        self.sigActivated2.emit(self)
        self.emitStateChanged('activated', None)

    def activate3(self):
        self.sigActivated3.emit(self)
        self.emitStateChanged('activated', None)

class SliderParameterItem(ParameterItem):
    # 'ButtonText' paramter is a tuple ('Off Text', 'On Text')
    def __init__(self, param, depth):
        ParameterItem.__init__(self, param, depth)
        self.layoutWidget = QtGui.QWidget()
        self.layout = QtGui.QHBoxLayout()
        self.layoutWidget.setLayout(self.layout)
        bt = param.opts['ButtonText']
        self.onOffText = bt
        self.button = QtGui.QPushButton(bt[0])
        self.button.setCheckable(True)
        self.button.setChecked(False)
        #self.button.setStyleSheet(QtCore.QString(
        #    "QPushButton {background-color: red;} \
        #    QPushButton:checked{background-color: green;} \
        #    QPushButton:pressed {background-color: red;}" \
        #    ))
        self.layout.addWidget(self.button)
        self.layout.setContentsMargins(2,2,2,2)
        self.layout.setSpacing(0)
        self.layout.addStretch()
        self.button.toggled.connect(self.buttonToggled)
        param.sigNameChanged.connect(self.paramRenamed)
        self.setText(0, '')
        param.setValue(False)
        
    def treeWidgetChanged(self):
        ParameterItem.treeWidgetChanged(self)
        tree = self.treeWidget()
        if tree is None:
            return
        
        tree.setFirstItemColumnSpanned(self, True)
        tree.setItemWidget(self, 0, self.layoutWidget)
        
    def paramRenamed(self, param, name):
        self.button.setText(name)
        
    def buttonToggled(self, checked):
        if checked:
            self.button.setText(self.onOffText[1])
        else:
            self.button.setText(self.onOffText[0])
        self.param.setValue(checked)
        self.param.activate(checked)
        

class SliderParameter(Parameter):
    """Used for displaying a button within the tree."""
    itemClass = SliderParameterItem
    sigActivated = QtCore.Signal(object, bool)
    
    def activate(self, checked):
        self.sigActivated.emit(self, checked)
        self.emitStateChanged('activated', None)


registerParameterType('action2', ActionParameter2, override=True)
registerParameterType('action3', ActionParameter3, override=True)
registerParameterType('toggle', SliderParameter, override=True)
