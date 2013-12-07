#!/xde

""" Module to make a proxy of the orcisir_ISIRController based on the orc framework.

orcisir_ISIRController can be found in "https://hotline.isir.upmc.fr/wsvn/EReval".
The operations proposed by the controller and the tasks managment may not be
well documented, and there may be lack of flexibility (e.g. no default values).

To overcome that, proxy methods are defined in a python class which derived from
xdefw.rtt.Task, with more flexibility on parameter definition.

Finally, task instances are separated from the controller to facilitate their use.
"""

#import deploy.deployer as ddeployer
import xdefw.rtt
import rtt_interface
import physicshelper

import swig_isir_controller as sic

from numpy import sqrt

import json

from time import time

################################################################################
################################################################################
# useful constant value
_WhatPartDict = {
    None           : sic.INTERNAL,
    "INTERNAL"     : sic.INTERNAL,
    sic.INTERNAL   : sic.INTERNAL,
    "FULL_STATE"   : sic.FULL_STATE,
    sic.FULL_STATE : sic.FULL_STATE,
    "FREE_FLYER"   : sic.FREE_FLYER,
    sic.FREE_FLYER : sic.FREE_FLYER,
}


_CartesianDofs = {
    None    : (True, sic.XYZ),
    "RXYZ"  : (True, sic.XYZ),
    "R"     : (True, None),
    "X"     : (False, sic.X),
    "Y"     : (False, sic.Y),
    "Z"     : (False, sic.Z),
    "XY"    : (False, sic.XY),
    "XZ"    : (False, sic.XZ),
    "YZ"    : (False, sic.YZ),
    "XYZ"   : (False, sic.XYZ),
    "RX"    : (True , sic.X),
    "RY"    : (True , sic.Y),
    "RZ"    : (True , sic.Z),
    "RXY"   : (True , sic.XY),
    "RXZ"   : (True , sic.XZ),
    "RYZ"   : (True , sic.YZ),
    "RXYZ"  : (True , sic.XYZ),
}


################################################################################
################################################################################
################################################################################
class ISIRController(xdefw.rtt.Task):
    """ Proxy of orcisir_ISIRController.
    """

    _NBISIRCTRL = 0

    def __init__(self, dynamic_model, robot_name, physic_agent, sync_connector=None, solver="quadprog", reduced_problem=False,      create_function_name="Create", controller_name="ISIRController"):
        """ Instantiate proxy of controller.

        :param dynamic_model: The dynamic model based on the controlled robot
        :type  dynamic_model: :class:`physicshelper.DynamicModel` or string
        :param string robot_name: the name given to the robot in the GVM.Robot instance.
        :param physic_agent: the physic agent which can the link the robot to the controller through the IConnectorRobotJointTorque & OConnectorRobotState.
        :param sync_connector: the synchronisation connector when this is required (in the WorldManager package, it is ``WorldManager.icsync``)
        :param string solver: Choose the internal solver; for now "quadprog" or "qld"
        :param bool reduced_problem: whether one want to solve the problem in **[ddq, torque, fc]** (True) or **[torque, fc]** (False)
        """
        super(ISIRController, self).__init__(rtt_interface.PyTaskFactory.CreateTask("ISIRController_Task_"+str(ISIRController._NBISIRCTRL)))
        ISIRController._NBISIRCTRL += 1

        self.robot_name     = robot_name

        #######################################
        # Create connection with physic agent #
        #######################################
        # create port in this orocos task
        self.q_port     = self.addCreateInputPort("q", "VectorXd", True)
        self.qdot_port  = self.addCreateInputPort("qdot", "VectorXd", True)
        self.Hroot_port = self.addCreateInputPort("Hroot", "Displacementd", True)
        self.Troot_port = self.addCreateInputPort("Troot", "Twistd", True)
        self.tau_port   = self.addCreateOutputPort("tau", "VectorXd")
        self.q_ok       = False
        self.qdot_ok    = False
        self.Hroot_ok   = False
        self.Troot_ok   = False
        self.q          = None
        self.qdot       = None
        self.Hroot      = None
        self.Troot      = None

        # create connector in the physical agent: out.connector for robot state, and in.connector for tau
        robotPrefix = robot_name+"_"
        physic_agent.s.Connectors.IConnectorRobotJointTorque.new(robot_name+".ic_torque", robotPrefix, robot_name)
        physic_agent.s.Connectors.OConnectorRobotState.new(robot_name+".oc_state", robotPrefix, robot_name)

        # set connection from physic to controller
        physic_agent.getPort(robotPrefix+"q").connectTo(self.getPort("q"))
        physic_agent.getPort(robotPrefix+"qdot").connectTo(self.getPort("qdot"))
        physic_agent.getPort(robotPrefix+"Hroot").connectTo(self.getPort("Hroot"))
        physic_agent.getPort(robotPrefix+"Troot").connectTo(self.getPort("Troot"))
#        physic_agent.getPort("contacts").connectTo(self.getPort("contacts"))

        # set connection from controller to physic
        if sync_connector is not None:
            sync_connector.addEvent(robotPrefix+"tau")
        self.getPort("tau").connectTo(physic_agent.getPort(robotPrefix+"tau"))

        ##################################
        # Set dynamic model of the robot #
        ##################################
        # set dynamic model and init inner solver
        if isinstance(dynamic_model, physicshelper.DynamicModel):
            self.dynamic_model = sic.getModelFromXDEDynamicModel(dynamic_model)
        elif isinstance(dynamic_model, basestring):
            self.dynamic_model = sic.getModelFromSharedLibrary(dynamic_model, create_function_name, robot_name) #self.s.setModelFromSharedLibrary(dynamic_model, createFunctionName, robot_name)
        else:
            self.dynamic_model = dynamic_model

        assert(isinstance(self.dynamic_model, sic.swig_isir_controller.Model)) # check if model is compatible with controller

        ##############
        # Set solver #
        ##############
        if isinstance(solver, basestring):
            if solver == "quadprog":
                self.solver = sic.OneLevelSolverWithQuadProg()
            elif solver == "qld":
                self.solver = sic.OneLevelSolverWithQLD()
        else:
            self.solver = solver

        assert(isinstance(self.solver, sic.swig_isir_controller.Solver)) # check if solver is compatible with controller

        self.controller = sic.ISIRController(controller_name, self.dynamic_model, self.solver, reduced_problem)

        self.registered_tasks       = []
        self.registered_constraints = []
        self.registered_updaters    = []


    def add_constraint(self, const):
        self.registered_constraints.append(const)
        self.controller.addConstraint(const)
        return const

    def remove_constraint(self, const):
        self.registered_constraints.remove(const)
        self.controller.removeConstraint(const)
        return const

    def add_updater(self, updater):
        self.registered_updaters.append(updater)
        return updater

    def remove_updater(self, updater):
        self.registered_updaters.remove(updater)
        return updater

    def startHook(self):
        self._perf_timeline        = []
        self._perf_model_update    = []
        self._perf_updaters_update = []
        self._perf_compute_output  = []

    def stopHook(self):
        pass


    def exceptionHook(self):
        print "IN EXCEPTION HOOK: exception raised, close python"
        exit() #close python harshly

    def updateHook(self):
        if self.q_ok     is False:
            self.q, self.q_ok         = self.q_port.read()
        if self.qdot_ok  is False:
            self.qdot, self.qdot_ok   = self.qdot_port.read()
        if self.Hroot_ok is False:
            self.Hroot, self.Hroot_ok = self.Hroot_port.read()
        if self.Troot_ok is False:
            self.Troot, self.Troot_ok = self.Troot_port.read()

        if self.q_ok and self.qdot_ok and self.Hroot_ok and self.Troot_ok:
            self.q_ok     = False
            self.qdot_ok  = False
            self.Hroot_ok = False
            self.Troot_ok = False

            self._perf_timeline.append(time())

            # update model
            _t = time()
            if self.dynamic_model.hasFixedRoot():
                self.dynamic_model.setState(self.q, self.qdot)
            else:
                self.dynamic_model.setState(self.Hroot, self.q, self.Troot, self.qdot)
            self._perf_model_update.append(time() -_t)

            # update updaters
            _t = time()
            for upd in self.registered_updaters:
                upd.update()
            self._perf_updaters_update.append(time() -_t)

            # compute output
            _t = time()
            tau = self.controller.computeOutput()
            self._perf_compute_output.append(time() - _t)

            self.tau_port.write(tau)



    def getModel(self):
        return self.dynamic_model

    def getController(self):
        return self.controller


    ################################
    # Get performances information #
    ################################
    def getPerformances(self):
        """ Get performances from the controller in a JSON style. """
        perf = json.loads(self.controller.getPerformances())
        perf.update({ "timeline"       : self._perf_timeline,
                      "model_update"   : self._perf_model_update,
                      "updaters_update": self._perf_updaters_update,
                      "compute_output" : self._perf_compute_output})
        return perf


    ########################
    # Set task constructor #
    ########################
    def _addRegisterTask_(self, ntask, mState, feat, tState=None, featDes=None, **kwargs):
        self.controller.addTask(ntask)
        itask = ISIRTask(ntask, mState, feat, tState, featDes, **kwargs)
        self.registered_tasks.append(itask)
        return itask


    def createFullTask(self, taskName, whatPart=None, model=None, **kwargs):
        whatPart = _WhatPartDict[whatPart]

        if model is None:
            model = self.dynamic_model

        FMS     = sic.FullModelState(  taskName+".FullModelState"      , model, whatPart)
        FTS     = sic.FullTargetState( taskName+".FullTargetState"     , model, whatPart)
        feat    = sic.FullStateFeature(taskName+".FullStateFeature"    , FMS)
        featDes = sic.FullStateFeature(taskName+".FullStateFeature_Des", FTS)

        fullTask = self.controller.createISIRTask(taskName, feat, featDes)
        fullTask.initAsAccelerationTask()
        return self._addRegisterTask_(fullTask, FMS, feat, FTS, featDes, **kwargs)


    def createPartialTask(self, taskName, dofs, whatPart=None, model=None, **kwargs):
        whatPart = _WhatPartDict[whatPart]
        if whatPart is sic.FREE_FLYER:
            raise KeyError, "free_flyer is not valid, use full_state instead."

        if model is None:
            model = self.dynamic_model

        sdofs = []
        for d in dofs:
            if isinstance(d, int):
                assert(d>=0)
                sdofs.append(d)
            elif isinstance(d, basestring):
                dinternal = self.dynamic_model.getSegmentIndex(d)
                if whatPart == sic.INTERNAL:
                    sdofs.append(dinternal)
                elif whatPart == sic.FULL_STATE:
                    decal = 0 if self.dynamic_model.hasFixedRoot() else 6
                    sdofs.append(dinternal+decal)

        PMS     = sic.PartialModelState(  taskName+".PartialModelState"      , model, sdofs, whatPart)
        PTS     = sic.PartialTargetState( taskName+".PartialTargetState"     , model, sdofs, whatPart)
        feat    = sic.PartialStateFeature(taskName+".PartialStateFeature"    , PMS)
        featDes = sic.PartialStateFeature(taskName+".PartialStateFeature_Des", PTS)

        partialTask = self.controller.createISIRTask(taskName, feat, featDes)
        partialTask.initAsAccelerationTask()
        return self._addRegisterTask_(partialTask, PMS, feat, PTS, featDes, **kwargs)


    def createFrameTask(self, taskName, segmentName, H_segment_frame, dofs=None, model=None, **kwargs):
        dofs = _CartesianDofs[dofs]

        if model is None:
            model = self.dynamic_model

        SF       = sic.SegmentFrame(taskName+".SegmentFrame", model, segmentName, H_segment_frame)
        TF       = sic.TargetFrame( taskName+".TargetFrame" , model)

        if   dofs[0]==True:
            if dofs[1] is None:
                feat    = sic.OrientationFeature(taskName+".OrientationFeature"    , SF)
                featDes = sic.OrientationFeature(taskName+".OrientationFeature_Des", TF)
            else:
                feat    = sic.DisplacementFeature(taskName+".DisplacementFeature"    , SF, dofs[1])
                featDes = sic.DisplacementFeature(taskName+".DisplacementFeature_Des", TF, dofs[1])
        else:
            feat    = sic.PositionFeature(taskName+".PositionFeature"    , SF, dofs[1])
            featDes = sic.PositionFeature(taskName+".PositionFeature_Des", TF, dofs[1])

        frameTask = self.controller.createISIRTask(taskName, feat, featDes)
        frameTask.initAsAccelerationTask()
        return self._addRegisterTask_(frameTask, SF, feat, TF, featDes, **kwargs)


    def createCoMTask(self, taskName, dofs="XYZ", model=None, **kwargs):
        dofs = _CartesianDofs[dofs]
        if dofs[0] == True:
            raise ValueError("Cannot control CoM in rotation.")
        if model is None:
            model = self.dynamic_model

        CoMF    = sic.CoMFrame(   taskName+".CoMFrame"   , model);
        TF      = sic.TargetFrame(taskName+".TargetFrame", model);
        feat    = sic.PositionFeature(taskName+".PositionFeature"    , CoMF, dofs[1]);
        featDes = sic.PositionFeature(taskName+".PositionFeature_Des", TF  , dofs[1]);

        CoMTask = self.controller.createISIRTask(taskName, feat, featDes)
        CoMTask.initAsAccelerationTask()
        return self._addRegisterTask_(CoMTask, CoMF, feat, TF, featDes, **kwargs)


    def createTorqueTask(self, taskName, dofs, model=None, **kwargs):
        # The dofs are necesseraly internal

        if model is None:
            model = self.dynamic_model

        PMS     = sic.PartialModelState(  taskName+".PartialModelState"      , model, dofs, sic.INTERNAL)
        PTS     = sic.PartialTargetState( taskName+".PartialTargetState"     , model, dofs, sic.INTERNAL)
        feat    = sic.PartialStateFeature(taskName+".PartialStateFeature"    , PMS)
        featDes = sic.PartialStateFeature(taskName+".PartialStateFeature_Des", PTS)

        torqueTask = self.controller.createISIRTask(taskName, feat, featDes)
        torqueTask.initAsTorqueTask()
        return self._addRegisterTask_(torqueTask, PMS, feat, PTS, featDes, **kwargs)



    def createForceTask(self, taskName, segmentName, H_segment_frame, model=None, **kwargs):
        if model is None:
            model = self.dynamic_model

        SF      = sic.SegmentFrame(taskName+".SegmentFrame", model, segmentName, H_segment_frame)
        TF      = sic.TargetFrame( taskName+".TargetFrame" , model)
        feat    = sic.PositionFeature(taskName+".PositionFeature"    , SF, sic.XYZ)
        featDes = sic.PositionFeature(taskName+".PositionFeature_Des", TF, sic.XYZ)

        forceTask = self.controller.createISIRTask(taskName, feat, featDes)
        forceTask.initAsForceTask()
        return self._addRegisterTask_(forceTask, SF, feat, TF, featDes, **kwargs)


    def createContactTask(self, taskName, segmentName, H_segment_frame, mu, margin=0, model=None, **kwargs):
        if model is None:
            model = self.dynamic_model

        SF   = sic.SegmentFrame(taskName+".SegmentFrame"          , model, segmentName, H_segment_frame)
        feat = sic.PointContactFeature(taskName+".PositionFeature", SF)

        contactTask = self.controller.createISIRContactTask(taskName, feat, mu, margin)
        contactTask.initAsAccelerationTask() # we control the acceleration of the contact point
        return self._addRegisterTask_(contactTask, SF, feat, **kwargs)



    def createGenericTask(self, taskName, taskType, modelState, modelFeature, targetState=None, targetFeature=None, **kwargs):
        if targetFeature is not None:
            genTask = self.controller.createISIRTask(taskName, modelFeature, targetFeature)
        else:
            genTask = self.controller.createISIRTask(taskName, modelFeature)

        if   taskType == "acceleration":
            genTask.initAsAccelerationTask()
        elif taskType == "torque":
            genTask.initAsTorqueTask()
        elif taskType == "force":
            genTask.initAsForceTask()
        else:
            raise ValueError("taskType '"+taskType+"' is invalid; It should be one of 'acceleration', 'torque', 'force'")

        return self._addRegisterTask_(genTask, modelState, modelFeature, targetState, targetFeature, **kwargs)










class ISIRTask(object):
    def __init__(self, task, modelState, modelFeature, targetState=None, targetFeature=None, **kwargs):
        self._task          = task              #
        self._modelState    = modelState        # create this class to keep a ref on state and features.
        self._modelFeature  = modelFeature      # if not, they are not saved, are deleted by the garbage collector
        self._targetState   = targetState       # and future call of the task will lead to error because new state & feature are alredy deleted.
        self._targetFeature = targetFeature     # TODO: maybe a better solution with the use of acquire()

        # To set the task method directly accessible by ISIRTask
        for fnName in dir(self._task):
            if fnName[:1] != "_" and hasattr(getattr(self._task, fnName), "__call__"):
                setattr(self, fnName, getattr(self._task, fnName))

        # Add targetState setter if any
        if self._targetState is not None:
            for fnName in dir(self._targetState):
                if fnName[:3] != "set":
                    setattr(self, fnName, getattr(self._targetState, fnName))

        if "kp" in kwargs:
            self.setStiffness(kwargs["kp"])
            self.setDamping(2*sqrt(kwargs["kp"]))
        if "kd" in kwargs:
            self.setDamping(kwargs["kd"])

        for k, v in kwargs.items():
            if k in ["weight", "w"]:
                self.setWeight(v)
            elif k in ["set_q", "q_des"]:
                self.set_q(v)
            elif k in ["set_qdot", "dq_des"]:
                self.set_qdot(v)
            elif k in ["setPosition", "pose_des"]:
                self.setPosition(v)
            elif k in ["setVelocity", "vel_des"]:
                self.setVelocity(v)

        self.activateAsObjective()


    def getTask(self):
        return self._task

    def getModelState(self):
        return self._modelState

    def getTargetState(self):
        return self._targetState







