from PySide import QtCore, QtGui
import IPython
import sys
import lgsm

import swig_isir_controller as sic

""" Enabling ipython with qt
:param app: Ignored parameter (use to uniform interface)
"""
def configure(app=None):
    IPython.lib.inputhook.enable_qt4(app)

class TaskGui(QtGui.QScrollArea):
    def __init__(self, task):
        super(TaskGui, self).__init__()
        self.task = task
        self.task_gui = QtGui.QWidget()

        if isinstance(self.task, sic.FullTargetState) or isinstance(self.task, sic.PartialTargetState):
            self.initJointTaskGui()

            self.setWidget(self.task_gui)
            self.setWindowTitle(self.task.getName())
            self.setGeometry(350, 350, 450, 500)
            self.show()
        else:
            raise ValueError("Unsupported task for gui")

    def _createSlider(self):
        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.setSingleStep(1)
        slider.setRange(-400, 400)
        return slider

    def initJointTaskGui(self):
        self.groupbox_joint_task = QtGui.QHBoxLayout()
        #check task is acceleration, torque or force
        if self.task.getTaskType() == sic.ACCELERATIONTASK:
            self.groupbox_q = QtGui.QGroupBox("q")
            self.groupbox_qdot = QtGui.QGroupBox("qdot")
            self.groupbox_qddot = QtGui.QGroupBox("qddot")
            taskDim = self.task.getDimension()
            self.qdes = lgsm.zeros(taskDim)

            self.task_gui.setGeometry(300, 300, 400*3, (1+taskDim)*40)
            gridlayout_q = QtGui.QGridLayout()
            gridlayout_qdot = QtGui.QGridLayout()
            gridlayout_qddot = QtGui.QGridLayout()

            for i in range(taskDim):
                label_task_joint_q = QtGui.QLabel(str(i))
                label_task_joint_qdot = QtGui.QLabel(str(i))
                label_task_joint_qddot = QtGui.QLabel(str(i))
                label_task_joint_value_q = QtGui.QLabel(str(0.0))
                label_task_joint_value_qdot = QtGui.QLabel(str(0.0))
                label_task_joint_value_qddot = QtGui.QLabel(str(0.0))
                slider_q = self._createSlider()
                slider_qdot = self._createSlider()
                slider_qddot = self._createSlider()

                gridlayout_q.addWidget(label_task_joint_q, i, 0)
                gridlayout_q.addWidget(label_task_joint_value_q, i, 1)
                gridlayout_q.addWidget(slider_q, i, 2)

                gridlayout_qdot.addWidget(label_task_joint_qdot, i, 0)
                gridlayout_qdot.addWidget(label_task_joint_value_qdot, i, 1)
                gridlayout_qdot.addWidget(slider_qdot, i, 2)

                gridlayout_qddot.addWidget(label_task_joint_qddot, i, 0)
                gridlayout_qddot.addWidget(label_task_joint_value_qddot, i, 1)
                gridlayout_qddot.addWidget(slider_qddot, i, 2)

            self.groupbox_q.setLayout(gridlayout_q)
            self.groupbox_qdot.setLayout(gridlayout_qdot)
            self.groupbox_qddot.setLayout(gridlayout_qddot)

            self.groupbox_joint_task.addWidget(self.groupbox_q)
            self.groupbox_joint_task.addWidget(self.groupbox_qdot)
            self.groupbox_joint_task.addWidget(self.groupbox_qddot)

            self.task_gui.setLayout(self.groupbox_joint_task)

def createTaskGui(task):
    app_created = False
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtGui.QApplication(sys.argv)
        app_created = True
    app.references = set()
    wid = TaskGui(task)
    app.references.add(wid)
    wid.show()
    if app_created:
        app.exec_()
    return wid
 
