#!/xde


import deploy.deployer as ddeployer
import dsimi.rtt

import lgsm





################################################################################
################################################################################
################################################################################
class ISIRCtrl(dsimi.rtt.Task):
    """
    """
    def __init__(self, libdir, dynamic_model, physic_agent, robotState_prefix, robotJointTorque_prefix, sync_connector=None, solver="quadprog"):
        """
        """
        orocos_ICTask = ddeployer.load("oIT", "Orocos_ISIRController",
                                       module="orcisir_Orocos_IsirController-gnulinux", prefix="",
                                       libdir=libdir)
        super(ISIRCtrl, self).__init__(orocos_ICTask)
        
        # set dynamic model and init inner solver
        self.dynamic_model = dynamic_model
        self.s.setDynModelPointerStr(str(self.dynamic_model.this.__long__()), solver) #or "qld" or "quadprog"
        
        self.NDOF0 = 6
        if self.dynamic_model.hasFixedRoot():
            self.NDOF0 = 0

        # set connection from physic to controller
        physic_agent.getPort(robotState_prefix+"q").connectTo(self.getPort("q"))
        physic_agent.getPort(robotState_prefix+"qdot").connectTo(self.getPort("qDot"))
        physic_agent.getPort(robotState_prefix+"Hroot").connectTo(self.getPort("Hroot"))
        physic_agent.getPort(robotState_prefix+"Troot").connectTo(self.getPort("Troot"))
        physic_agent.getPort("contacts").connectTo(self.getPort("contacts"))

        # set connection from controller to physic
        if sync_connector is not None:
            sync_connector.addEvent(robotJointTorque_prefix+"tau")
        self.getPort("tau").connectTo(physic_agent.getPort(robotJointTorque_prefix+"tau"))



    ########################################################################
    # Methods to easily access operations saved in the ISIRController Task #
    ########################################################################
    def setJointLimits(self, lower_bounds, upper_bounds):
        """
        """
        self.s.setJointLimits(lgsm.vector(lower_bounds), lgsm.vector(upper_bounds))

    def setHorizonOfPrediction(self, horizon):
        """
        """
        self.s.setHorizonOfPrediction(horizon)

    def setTorqueLimits(self, torque_limits):
        """
        """
        self.s.setTorqueLimits(lgsm.vector(torque_limits))


    ##################################
    # Methods to easily create tasks #
    ##################################
    def createFullTask(self, name, weight=1., level=0):
        """
        """
        self.s.createFullTask(name)
        return ISIRTask(self, name, ISIRTask.FULLTASK, weight, level)

    def createPartialTask(self, name, dofs, weight=1., level=0):
        """
        """
        dofs_index = []
        for d in dofs:
            if isinstance(d, int):
                dofs_index.append(d)
            else:
                d_idx = self.dynamic_model.getSegmentIndex(d)
                dofs_index.append(self.NDOF0 + d_idx)

        self.s.createPartialTask(name, dofs_index)
        return ISIRTask(self, name, ISIRTask.PARTIALTASK, weight, level)

    def createFrameTask(self, name, segmentName, H_segment_frame, dofs, weight=1., level=0):
        """
        """
        self.s.createFrameTask(name, segmentName, lgsm.Displacement(H_segment_frame), dofs)
        return ISIRTask(self, name, ISIRTask.FRAMETASK, weight, level)

    def createCoMTask(self, name, dofs, weight=1., level=0):
        """
        """
        self.s.createCoMTask(name, dofs)
        return ISIRTask(self, name, ISIRTask.COMTASK, weight, level)

    def createContactTask(self, name, segment, H_segment_frame, mu, margin, weight=1., level=0):
        """
        """
        self.s.createContactTask(name, segment, H_segment_frame, mu, margin)
        return ISIRTask(self, name, ISIRTask.CONTACTTASK, weight, level)



################################################################################
################################################################################
################################################################################

class ISIRTask(object):
    """
    """
    
    FULLTASK    = "fullTask"
    PARTIALTASK = "partialTask"
    FRAMETASK   = "frameTask"
    COMTASK     = "CoMTask"
    CONTACTTASK = "contactTask"
    
    def __init__(self, ctrl, name, taskType, weight=1., level=0):
        """
        """
        self.ctrl = ctrl
        self.name = name
        self.taskType = taskType
        
        self.setWeight(weight)
        self.setLevel(level)
        
        self.dimension = self.ctrl.s.getTaskDimension(self.name)
        
        self.updateTaskFunction = None
        if taskType   == self.FULLTASK:
            self.updateTaskFunction = self.ctrl.s.updateFullTask
        elif taskType == self.PARTIALTASK:
            self.updateTaskFunction = self.ctrl.s.updatePartialTask
        elif taskType == self.FRAMETASK:
            self.updateTaskFunction = self.ctrl.s.updateFrameTask
        elif taskType == self.COMTASK:
            self.updateTaskFunction = self.ctrl.s.updateCoMTask

    def setKpKd(self, kp, kd=None):
        if kd is None:
            kd = 2.*lgsm.math.sqrt(kp)
        self.ctrl.s.setTaskKpKd(self.name, kp, kd)

    def setWeight(self, weight):
        self.ctrl.s.setTaskWeight(self.name, weight)

    def setLevel(self, level):
        self.ctrl.s.setTaskLevel(self.name, level)

    def activateAsObjective(self):
        self.ctrl.s.activateTaskAsObjective(self.name)

    def activateAsConstraint(self):
        self.ctrl.s.activateTaskAsConstraint(self.name)

    def deactivate(self):
        self.ctrl.s.deactivateTask(self.name)

    def update(self, posDes, velDes, accDes=None):
        if accDes is None:
            accDes = lgsm.zero(self.dimension)
        self.updateTaskFunction(self.name, posDes, velDes, accDes)


    def getError(self):
        return self.ctrl.s.getTaskError(self.name)

    def getErrorDot(self):
        return self.ctrl.s.getTaskErrorDot(self.name)



