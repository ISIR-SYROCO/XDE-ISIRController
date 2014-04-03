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

"""
Gui Hierarchy:
TaskGui <ScrollArea>
--groupbox_joint_task <QHBoxLayout>
----groupbox_common <QGroupBox>
------gridlayout_common <QGridLayout>
--------weight_label|weight_value_label|weight_slider
--------kp_label|kp_value_label|kp_slider
--------kd_label|kd_value_label|kd_slider
----groupbox_q <QGroupBox>
------gridlayout_q <QGridLayout>
--------label_task_joint_q|label_task_joint_value_q|slider_q
----groupbox_qdot <QGroupBox>
------gridlayout_qdot <QGridLayout>
--------label_task_joint_qdot|label_task_joint_value_qdot|slider_qdot
----groupbox_qddot <QGroupBox>
------gridlayout_qddot <QGridLayout>
--------label_task_joint_qddot|label_task_joint_value_qddot|slider_qddot
"""
class TaskGui(QtGui.QScrollArea):
    def __init__(self, task):
        super(TaskGui, self).__init__()
        self.task = task
        self.task_gui = QtGui.QWidget()
        self.groupbox_joint_task = QtGui.QHBoxLayout()

        if isinstance(self.task, sic.FullTargetState) or isinstance(self.task, sic.PartialTargetState):
            self.initJointTaskGui()

            self.setWidget(self.task_gui)
            self.setWindowTitle(self.task.getName())
            self.setGeometry(350, 350, 450, 500)
            self.show()
        else:
            raise ValueError("Unsupported task for gui")

    def _createSlider(self, step=1, slider_range=[-400, 400]):
        if len(slider_range) != 2:
            raise ValueError("Incorrect range size while creating slider")
        else:
            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setSingleStep(step)
            slider.setRange(slider_range[0], slider_range[1])
            return slider

    def _initCommonSlider(self):
        self.groupbox_common = QtGui.QGroupBox("Common")

        self.groupbox_joint_task.addWidget(self.groupbox_common)
        self.gridlayout_common = QtGui.QGridLayout()
        self.weight_label = QtGui.QLabel("Weight")
        self.kp_label = QtGui.QLabel("kp")
        self.kd_label = QtGui.QLabel("kd")

        self.weight_value_label = QtGui.QLabel()
        self.kp_value_label = QtGui.QLabel()
        self.kd_value_label = QtGui.QLabel()

        self.weight_slider = self._createSlider(slider_range=[1, 1000])
        self.weight_slider.valueChanged.connect(self.setWeight)
        self.kp_slider = self._createSlider(slider_range=[0, 3000])
        self.kp_slider.valueChanged.connect(self.setKp)
        self.kd_slider = self._createSlider(slider_range=[0, 3000])
        self.kd_slider.valueChanged.connect(self.setKd)

        self.gridlayout_common.addWidget(self.weight_label, 0, 0)
        self.gridlayout_common.addWidget(self.weight_value_label, 0, 1)
        self.gridlayout_common.addWidget(self.weight_slider, 0, 2)

        self.gridlayout_common.addWidget(self.kp_label, 1, 0)
        self.gridlayout_common.addWidget(self.kp_value_label, 1, 1)
        self.gridlayout_common.addWidget(self.kp_slider, 1, 2)

        self.gridlayout_common.addWidget(self.kd_label, 2, 0)
        self.gridlayout_common.addWidget(self.kd_value_label, 2, 1)
        self.gridlayout_common.addWidget(self.kd_slider, 2, 2)

        self.groupbox_common.setLayout(self.gridlayout_common)

    def initControlFrameTaskGui(self):
        self._initCommonSlider()

    def initJointTaskGui(self):
        self._initCommonSlider()
        taskDim = self.task.getDimension()
        #check task is acceleration, torque or force
        if self.task.getTaskType() == sic.ACCELERATIONTASK:
            self.groupbox_q = QtGui.QGroupBox("q")
            self.groupbox_qdot = QtGui.QGroupBox("qdot")
            self.groupbox_qddot = QtGui.QGroupBox("qddot")

            self.qdes = self.task.q()
            self.qdotdes = self.task.qdot()
            self.qddotdes = self.task.qddot()

            self.task_joint_q_sigmap_slider = QtCore.QSignalMapper(self)
            self.task_joint_q_sigmap_label = QtCore.QSignalMapper(self)

            self.task_joint_qdot_sigmap_slider = QtCore.QSignalMapper(self)
            self.task_joint_qdot_sigmap_label = QtCore.QSignalMapper(self)

            self.task_joint_qddot_sigmap_slider = QtCore.QSignalMapper(self)
            self.task_joint_qddot_sigmap_label = QtCore.QSignalMapper(self)

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
                slider_q.valueChanged.connect(self.task_joint_q_sigmap_slider.map)
                self.task_joint_q_sigmap_slider.setMapping(slider_q, i)
                self.task_joint_q_sigmap_label.setMapping(label_task_joint_value_q, i)

                gridlayout_qdot.addWidget(label_task_joint_qdot, i, 0)
                gridlayout_qdot.addWidget(label_task_joint_value_qdot, i, 1)
                gridlayout_qdot.addWidget(slider_qdot, i, 2)
                slider_qdot.valueChanged.connect(self.task_joint_qdot_sigmap_slider.map)
                self.task_joint_qdot_sigmap_slider.setMapping(slider_qdot, i)
                self.task_joint_qdot_sigmap_label.setMapping(label_task_joint_value_qdot, i)

                gridlayout_qddot.addWidget(label_task_joint_qddot, i, 0)
                gridlayout_qddot.addWidget(label_task_joint_value_qddot, i, 1)
                gridlayout_qddot.addWidget(slider_qddot, i, 2)
                slider_qddot.valueChanged.connect(self.task_joint_qddot_sigmap_slider.map)
                self.task_joint_qddot_sigmap_slider.setMapping(slider_qddot, i)
                self.task_joint_qddot_sigmap_label.setMapping(label_task_joint_value_qddot, i)

            self.groupbox_q.setLayout(gridlayout_q)
            self.groupbox_qdot.setLayout(gridlayout_qdot)
            self.groupbox_qddot.setLayout(gridlayout_qddot)

            self.groupbox_joint_task.addWidget(self.groupbox_q)
            self.groupbox_joint_task.addWidget(self.groupbox_qdot)
            self.groupbox_joint_task.addWidget(self.groupbox_qddot)

            self.task_joint_q_sigmap_slider.mapped.connect(self.setQdes)
            self.task_joint_qdot_sigmap_slider.mapped.connect(self.setQdotdes)
            self.task_joint_qddot_sigmap_slider.mapped.connect(self.setQddotdes)
            self.task_gui.setLayout(self.groupbox_joint_task)
            self.syncDes()

        elif self.task.getTaskType() == sic.TORQUETASK:
            self.groupbox_tau = QtGui.QGroupBox("tau")

            self.taudes = self.task.tau()

            self.task_joint_tau_sigmap_slider = QtCore.QSignalMapper(self)
            self.task_joint_tau_sigmap_label = QtCore.QSignalMapper(self)

            self.task_gui.setGeometry(300, 300, 400, (1+taskDim)*60)
            gridlayout_tau = QtGui.QGridLayout()

            for i in range(taskDim):
                label_task_joint_tau = QtGui.QLabel(str(i))
                label_task_joint_value_tau = QtGui.QLabel(str(0.0))
                slider_tau = self._createSlider(slider_range=[-1000, 1000])

                gridlayout_tau.addWidget(label_task_joint_tau, i, 0)
                gridlayout_tau.addWidget(label_task_joint_value_tau, i, 1)
                gridlayout_tau.addWidget(slider_tau, i, 2)
                slider_tau.valueChanged.connect(self.task_joint_tau_sigmap_slider.map)
                self.task_joint_tau_sigmap_slider.setMapping(slider_tau, i)
                self.task_joint_tau_sigmap_label.setMapping(label_task_joint_value_tau, i)

            self.groupbox_tau.setLayout(gridlayout_tau)
            self.groupbox_joint_task.addWidget(self.groupbox_tau)

            self.task_joint_tau_sigmap_slider.mapped.connect(self.setTaudes)
            self.task_gui.setLayout(self.groupbox_joint_task)

        self.syncCommon()

    def syncDes(self):
        for i in range(self.task.getDimension()):
            self.task_joint_q_sigmap_slider.mapping(i).setValue(int(self.qdes[i]*100))
            self.task_joint_q_sigmap_label.mapping(i).setText("[%.2f]" % self.qdes[i])

            self.task_joint_qdot_sigmap_slider.mapping(i).setValue(int(self.qdotdes[i]*100))
            self.task_joint_qdot_sigmap_label.mapping(i).setText("[%.2f]" % self.qdotdes[i])

            self.task_joint_qddot_sigmap_slider.mapping(i).setValue(int(self.qddotdes[i]*100))
            self.task_joint_qddot_sigmap_label.mapping(i).setText("[%.2f]" % self.qddotdes[i])

    def syncCommon(self):
        weight = self.task.getWeight()[0]
        self.weight_value_label.setText("[%.3f]" % weight) 
        self.weight_slider.setValue(int(weight*1000))

        kp = self.task.getStiffness()[0, 0]
        self.kp_value_label.setText("[%.2f]" % kp) 
        self.kp_slider.setValue(int(kp*100))

        kd = self.task.getDamping()[0, 0]
        self.kd_value_label.setText("[%.2f]" % kd) 
        self.kd_slider.setValue(int(kd*100))

    def setQdes(self, id):
        self.qdes[id] = self.task_joint_q_sigmap_slider.mapping(id).value()/100.0
        self.task.set_q(self.qdes)
        self.task_joint_q_sigmap_label.mapping(id).setText("[%.2f]" % self.qdes[id])

    def setTaudes(self, id):
        self.taudes[id] = self.task_joint_tau_sigmap_slider.mapping(id).value()/100.0
        self.task.set_tau(self.taudes)
        self.task_joint_tau_sigmap_label.mapping(id).setText("[%.2f]" % self.taudes[id])

    def setQdotdes(self, id):
        self.qdotdes[id] = self.task_joint_qdot_sigmap_slider.mapping(id).value()/100.0
        self.task.set_qdot(self.qdotdes)
        self.task_joint_qdot_sigmap_label.mapping(id).setText("[%.2f]" % self.qdotdes[id])

    def setQddotdes(self, id):
        self.qddotdes[id] = self.task_joint_qddot_sigmap_slider.mapping(id).value()/100.0
        self.task.set_qddot(self.qddotdes)
        self.task_joint_qddot_sigmap_label.mapping(id).setText("[%.2f]" % self.qddotdes[id])

    def setWeight(self):
        weight = self.weight_slider.value()/1000.0
        self.weight_value_label.setText("[%.3f]" % weight)
        self.task.setWeight(weight)

    def setKp(self):
        kp = self.kp_slider.value()/100.0
        self.kp_value_label.setText("[%.2f]" % kp)
        self.task.setStiffness(kp)

    def setKd(self):
        kd = self.kd_slider.value()/100.0
        self.kd_value_label.setText("[%.2f]" % kd)
        self.task.setDamping(kd)

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
 
