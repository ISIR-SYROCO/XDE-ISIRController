#!/xde

""" Module to make a proxy of the orcisir_ISIRController based on the orc framework.

orcisir_ISIRController can be found in "https://hotline.isir.upmc.fr/wsvn/EReval?".
The operations proposed by the controller and the tasks managment may not be
well documented, and there may be lack of flexibility (e.g. no default values).

To overcome that, proxy methods are defined in a python class which derived from
dsmi.rtt.Task, with more flexibility on parameter definition.

Finally, task instances are separated from the controller to facilitate their use.
"""

import deploy.deployer as ddeployer
import dsimi.rtt
import rtt_interface

import lgsm

import json



################################################################################
################################################################################
################################################################################
class ISIRCtrl(dsimi.rtt.Task):
    """ Proxy of orcisir_ISIRController.
    """

    def __init__(self, libdir, dynamic_model, robot_name, physic_agent, sync_connector=None, solver="quadprog", reduced_problem=False, multi_level=False):
        """ Instantiate proxy of controller.
        
        :param libdir: path string where one can find the lib 'orcisir_Orocos_IsirController-gnulinux'
        :param dynamic_model: xde.DynamicModel instance built based on the controlled robot
        :param robot_name: the name given to the robot in the GVM.Robot instance.
        :param physic_agent: the physic agent which can the link the robot to the controller through the IConnectorRobotJointTorque & OConnectorRobotState.
        :param sync_connector: the synchronisation connector when this is required (in the WorldManager package, it is WorldManager.icsync)
        :param solver: a string to choose the internal solve; for now "quadprog" or "qld"
        :param reduced_problem: whether one want to solve the problem in [ddq, torque, fc] (True) or [torque, fc] (False)
        :param multi_level: whether one want to solve multi level problem. Thus, it enables setLevel method for tasks.
        """
        orocos_ICTask = ddeployer.load("oIT", "Orocos_ISIRController",
                                       module="orcisir_Orocos_IsirController-gnulinux", prefix="",
                                       libdir=libdir)
        super(ISIRCtrl, self).__init__(orocos_ICTask)
        
        # set dynamic model and init inner solver
        self.dynamic_model = dynamic_model
        self.s.setDynModelPointerStr(str(self.dynamic_model.this.__long__()), solver, reduced_problem, multi_level)
        
        self.NDOF0 = 6
        if self.dynamic_model.hasFixedRoot():
            self.NDOF0 = 0

        # create connector in the physical agent: out.connector for robot state, and in.connector for tau
        robotPrefix = robot_name+"_"
        physic_agent.s.Connectors.IConnectorRobotJointTorque.new(robot_name+".ic_torque", robotPrefix, robot_name)
        physic_agent.s.Connectors.OConnectorRobotState.new(robot_name+".oc_state", robotPrefix, robot_name)

        # set connection from physic to controller
        physic_agent.getPort(robotPrefix+"q").connectTo(self.getPort("q"))
        physic_agent.getPort(robotPrefix+"qdot").connectTo(self.getPort("qdot"))
        physic_agent.getPort(robotPrefix+"Hroot").connectTo(self.getPort("Hroot"))
        physic_agent.getPort(robotPrefix+"Troot").connectTo(self.getPort("Troot"))
        physic_agent.getPort("contacts").connectTo(self.getPort("contacts"))

        # set connection from controller to physic
        if sync_connector is not None:
            sync_connector.addEvent(robotPrefix+"tau")
        self.getPort("tau").connectTo(physic_agent.getPort(robotPrefix+"tau"))

        self.task_updater = ISIRTaskUpdater()
        self.getPort("tasks_to_update").connectTo(self.task_updater.getPort("ctrl_trigger"))
        self.task_updater.getPort("tasks_updated").connectTo(self.getPort("tasks_updated"))
        self.task_updater.s.start()


    ########################################################################
    # Methods to easily access operations saved in the ISIRController Task #
    ########################################################################
    def setJointLimits(self, lower_bounds, upper_bounds):
        """ Set the joint limits for the associated constraint.
        
        :param lower_bounds: list of lower limits, dim=model.nbInternalDofs()
        :param upper_bounds: list of upper limits, dim=model.nbInternalDofs()
        """
        self.s.setJointLimits(lgsm.vector(lower_bounds), lgsm.vector(upper_bounds))

    def setJointLimitsHorizonOfPrediction(self, horizon):
        """ Set the horizon of prediction (in second) for the joint limit constraint. """
        self.s.setJointLimitsHorizonOfPrediction(horizon)

    def setTorqueLimits(self, torque_limits):
        """ Set the torque limits for the associated constraint.
        
        :param torque_limits: torque limits, dim=model.nbInternalDofs()
        """
        self.s.setTorqueLimits(lgsm.vector(torque_limits))

    def setContactAvoidanceHorizonOfPrediction(self, horizon):
        """ Set the horizon of prediction (in second) for the contact avoidance constraint. """
        self.s.setContactAvoidanceHorizonOfPrediction(horizon)

    def setContactAvoidanceMargin(self, margin):
        """ Set the horizon of prediction (in second) for the contact avoidance constraint. """
        self.s.setContactAvoidanceMargin(margin)

    def setFixedRootPosition(self, H_root):
        """ Set the root position when the robot has a fixed base.
        
        When you change the root position of a fixed robot with method 'set_H_root_bm' for instance,
        the modification is not taken into account in your dynamic model saved in the controller.
        This can lead to miscalculation when updating task and constraints.
        To take into account this modification, you must update your model with method 'setFreeFlyerPosition',
        or you must use this method.
        
        :param H_root: a lgsm.Displacement which represent the root position from ground
        """
        self.s.setFixedRootPosition(H_root)



    ##################################
    # Methods to easily create tasks #
    ##################################
    def createFullTask(self, name, weight=1., whichPart="INTERNAL", **kwargs):
        """ Create a task that control the full state of the model.
        
        Generally, to set a reference posture of the robot.
        
        :param name: the unique name (id) of the task
        :param weight: the task weight for control trade-off when some tasks are conflicting
        :param whichPart: tell what to control, see below
        :param kwargs: some keyword arguments to quickly initialize task. see ISIRTask.__init__ for more info.
        
        :rtype: a ISIRTask instance which give access to the task methods and bypass the controller
        
        A full task can control different parts of the robot which are:

        * "INTERNAL"   -> internal joints of the robot
        * "FREE_FLYER" -> the free flying pose of the robot (6 dofs) if any; the desired position is then
                          a lgsm.vector (dim=6) representing the 3 positions (x,y,z) and the 3 components
                          of the quaternion axis (wx, wy, wz) (the imaginary part). #TODO: check if this is the good representation
        * "FULL_STATE" -> control the free flying dofs and internal joints of the robot.
                          if the robot has a fixed base, "INTERNAL" and "FULL_STATE" are equivalent
        """
        index = self.s.createFullTask(name, whichPart)
        return ISIRTask(self, name, index, ISIRTask.FULLTASK, weight, **kwargs)

    def createPartialTask(self, name, dofs, weight=1., **kwargs):
        """ Create a task that control some state of the model.
        
        Generally, to control a particular subset of the robot, e.g. the arm, the leg, the spine...
        
        :param name: the unique name (id) of the task
        :param dofs: list of int (segment index) or string (segment name) corresponding to the controlled dofs.
                     Note that if you use a list of int, you must shift the segment indexes by 6 when the robot has a free flying root.
                     It also means that the free floating pose can be controlled.
        :param weight: the task weight for control trade-off when some tasks are conflicting
        :param kwargs: some keyword arguments to quickly initialize task. see ISIRTask.__init__ for more info.
        
        :rtype: a ISIRTask instance which give access to the task methods and bypass the controller
        """
        dofs_index = []
        for d in dofs:
            if isinstance(d, int):
                dofs_index.append(d)
            else:
                d_idx = self.dynamic_model.getSegmentIndex(d)
                dofs_index.append(self.NDOF0 + d_idx)

        index = self.s.createPartialTask(name, dofs_index)
        return ISIRTask(self, name, index, ISIRTask.PARTIALTASK, weight, **kwargs)

    def createFrameTask(self, name, segmentName, H_segment_frame, dofs, weight=1., **kwargs):
        """ Create a task that control a frame of the model.
        
        Generally to track a pose or a trajectory in the cartesian space.
        
        :param name: the unique name (id) of the task
        :param segmentName: the segment name that is rigidly linked with the controlled frame
        :param H_segment_frame: the lgsm.Displacement from the origin of the segment to the frame
        :param dofs: a string representing the controlled part of the frame, e.g. the rotation or the X & Y axis.
                     dofs is the combination of the following character (in this order): 'R', 'X', 'Y', 'Z'
        :param weight: the task weight for control trade-off when some tasks are conflicting
        :param kwargs: some keyword arguments to quickly initialize task. see ISIRTask.__init__ for more info.
        
        :rtype: a ISIRTask instance which give access to the task methods and bypass the controller
        """
        index = self.s.createFrameTask(name, segmentName, lgsm.Displacement(H_segment_frame), dofs.upper())
        return ISIRTask(self, name, index, ISIRTask.FRAMETASK, weight, **kwargs)

    def createCoMTask(self, name, dofs, weight=1., **kwargs):
        """ Create a task that control the Center of Mass (CoM) of the model.
        
        Generally associated to any balancing control, walking, static equilibrium etc...
        
        :param name: the unique name (id) of the task
        :param dofs: a string representing the controlled part of the CoM (here rotation control is meaningless)
                     it is the combination of the following character (in this order): 'X', 'Y', 'Z'
        :param weight: the task weight for control trade-off when some tasks are conflicting
        :param kwargs: some keyword arguments to quickly initialize task. see ISIRTask.__init__ for more info.
        
        :rtype: a ISIRTask instance which give access to the task methods and bypass the controller
        """
        index = self.s.createCoMTask(name, dofs.upper())
        return ISIRTask(self, name, index, ISIRTask.COMTASK, weight, **kwargs)

    def createContactTask(self, name, segmentName, H_segment_frame, mu, margin=0., weight=1., **kwargs):
        """ Create a task for frictional interaction with the environment.
        
        :param name: the unique name (id) of the task
        :param segmentName: the segment name that is rigidly linked with the contact frame
        :param H_segment_frame: the lgsm.Displacement from the origin of the segment to the frame
        :param mu: the Coulomb coefficient of friction
        :param margin: margin associated to the friction cone constraint. Positive margin means a thiner cone.
        :param weight: the task weight for control trade-off when some tasks are conflicting
        :param kwargs: some keyword arguments to quickly initialize task. see ISIRTask.__init__ for more info.
        
        :rtype: a ISIRTask instance which give access to the task methods and bypass the controller
        """
        index = self.s.createContactTask(name, segmentName, lgsm.Displacement(H_segment_frame), mu, margin)
        return ISIRTask(self, name, index, ISIRTask.CONTACTTASK, weight, **kwargs)



    ###################################
    # Methods for contact information #
    ###################################
    def addContactInformation(self, phy_outContactPort, ctrl_inPortName, segmentName):
        """ Add contact information in the solver to update contact task/constraints.
        
        :param phy_outContactPort: the dsimi.rtt.OutputPort which will transmit the contact information from physic agent
        :param ctrl_inPortName: the name of the input port that will receive the contact information into the controller
        :param segmentName: the segment name on which applies the contact information
        """
        self.s.addContactInformation(ctrl_inPortName, segmentName)
        phy_outContactPort.connectTo(self.getPort(ctrl_inPortName))

    def useContactInformation(self, ctrl_inPortName, isUsed):
        """ Define if a contact information port is used for obstacle avoidance.
        
        :param ctrl_inPortName: the name of the input port that will receive the contact information into the controller
        :param isUsed: enable/disable contact information
        """
        self.s.useContactInformation(ctrl_inPortName, isUsed)

    ##########################
    # Enable/Disable methods #
    ##########################
    def enableJointLimits(self, enabled):
        """ Enable/disable the joint limits constraint. """
        self.s.enableJointLimits(enabled)

    def enableTorqueLimits(self, enabled):
        """ Enable/disable the torque limits constraint. """
        self.s.enableTorqueLimits(enabled)

    def enableContactAvoidance(self, enabled):
        """ Enable/disable the contact avoidance constraint. """
        self.s.enableContactAvoidance(enabled)

    ################################
    # Get performances information #
    ################################
    def getPerformances(self):
        """ Get performances from the controller in a JSON style. """
        return json.loads(self.s.getPerformances())

    def writePerformancesInFile(self, fileName):
        """ Write performances from the controller in a JSON file. """
        self.s.writePerformancesInFile(fileName)




################################################################################
################################################################################
################################################################################

class ISIRTask(object):
    """ Proxy of ISIRTask defined in the module 'orcisir_Orocos_IsirController-gnulinux'.
    
    This allows to bypass the controller when updating the task. A task instance which is
    separated from the controller gives simpler methods and leads to clearer code.
    """
    
    FULLTASK    = "fullTask"
    PARTIALTASK = "partialTask"
    FRAMETASK   = "frameTask"
    COMTASK     = "CoMTask"
    CONTACTTASK = "contactTask"
    
    def __init__(self, ctrl, name, index, taskType, weight=1., **kwargs):
        """ Instantiate a proxy of an ISIRTask.
        
        Warning: when creating this proxy the task must have been registred by the controller before.
        
        :param ctrl: the controller in which the task has been registered
        :param name: the unique name (id) of the task
        :param taskType: the ISIRTask.TYPE of the task, which can be one of the following:
                         FULLTASK, PARTIALTASK, FRAMETASK, COMTASK, CONTACTTASK
        :param weight: the task weight for control trade-off when some tasks are conflicting
        
        :param kwargs: some keyword arguments can be pass to quickly initialize task parameters.
                       these arguments can be any of: level, kp, kd, pos_des, vel_des, acc_des.
        """
        self.ctrl     = ctrl
        self.name     = name
        self.index    = index
        self.taskType = taskType
        
        print "init TASK", name, ":", index
        
        self.setWeight(weight)
        
        self.dimension = self.ctrl.s.getTaskDimension(self.index)
        
        self.updateTaskFunction = None
        self.null_pos_des       = None
        self.null_vel_des       = None
        
        if taskType   == self.FULLTASK:
            self.updateTaskFunction = self.ctrl.s.updateFullTask
            self.null_pos_des       = lgsm.zero(self.dimension)
            self.null_vel_des       = lgsm.zero(self.dimension)

        elif taskType == self.PARTIALTASK:
            self.updateTaskFunction = self.ctrl.s.updatePartialTask 
            self.null_pos_des       = lgsm.zero(self.dimension)
            self.null_vel_des       = lgsm.zero(self.dimension)

        elif taskType == self.FRAMETASK:
            self.updateTaskFunction = self.ctrl.s.updateFrameTask
            self.null_pos_des       = lgsm.Displacement()
            self.null_vel_des       = lgsm.Twist()

        elif taskType == self.COMTASK:
            self.updateTaskFunction = self.ctrl.s.updateCoMTask
            self.null_pos_des       = lgsm.Displacement()
            self.null_vel_des       = lgsm.Twist()

        #treat kwargs arguments
        if ("level" in kwargs):
            self.setLevel(kwargs["level"])

        if ("kp" in kwargs) or ("kd" in kwargs):
            kp = kwargs["kp"] if "kp" in kwargs else 0.
            kd = kwargs["kd"] if "kd" in kwargs else None
            self.setKpKd(kp, kd)

        if ("pos_des" in kwargs) or ("vel_des" in kwargs) or ("acc_des" in kwargs):
            pos_des = kwargs["pos_des"] if "pos_des" in kwargs else self.null_pos_des
            vel_des = kwargs["vel_des"] if "vel_des" in kwargs else self.null_vel_des
            acc_des = kwargs["acc_des"] if "acc_des" in kwargs else self.null_vel_des
            self.update(pos_des, vel_des, acc_des)


    def setKpKd(self, kp, kd=None):
        """ Set the proportionnal (kp) and derivative (kd) gains of the task.
        
        :param kp: the proportionnal gain
        :param kd: the derivative gain. Note that if kd is None, then kd become 2.*sqrt(kp)
        """
        if kd is None:
            kd = 2.*lgsm.math.sqrt(kp)
        self.ctrl.s.setTaskKpKd(self.index, kp, kd)

    def setWeight(self, weight):
        """ Set task weight """
        self.ctrl.s.setTaskWeight(self.index, weight)

    def setLevel(self, level):
        """ Set task level """
        self.ctrl.s.setTaskLevel(self.index, level)

    def activateAsObjective(self):
        """ Set task as an objective, meaning that an error may occur """
        self.ctrl.s.activateTaskAsObjective(self.index)

    def activateAsConstraint(self):
        """ Set task as a constraint, meaning that no error should occur """
        self.ctrl.s.activateTaskAsConstraint(self.index)

    def deactivate(self):
        """ Deactive objectives and constraints linked to the task """
        self.ctrl.s.deactivateTask(self.index)

    def update(self, posDes, velDes=None, accDes=None):
        """ Update the desired values tracked by the task.
        
        :param posDes: a lgsm.Displacement or lgsm.vector representing the desired pose, depending on the taskType
        :param velDes: a lgsm.Twist or lgsm.vector representing the desired velocity, depending on the taskType
        :param accDes: a lgsm.Twist or lgsm.vector representing the reference acceleration, depending on the taskType.
                       If accDes is None, reference acceleration becomes null.
        """
        if velDes is None:
            velDes = self.null_vel_des
        if accDes is None:
            accDes = self.null_vel_des
        self.updateTaskFunction(self.index, posDes, velDes, accDes)


    def getError(self):
        """ get the tracking proportional error (the position error)
        
        :rtype: a lgsm.vector of position error
        """
        return self.ctrl.s.getTaskError(self.index)

    def getErrorDot(self):
        """ get the tracking derivative error (the velocity error)
        
        :rtype: a lgsm.vector of velocity error
        """
        return self.ctrl.s.getTaskErrorDot(self.index)



################################################################################
################################################################################
################################################################################
class ISIRTaskController(object):
    def __init__(self):
        pass

    def update(self, tick):
        pass


class ISIRTaskUpdater(dsimi.rtt.Task):
    """
    """
    def __init__(self):
        """
        """
        super(ISIRTaskUpdater, self).__init__(rtt_interface.PyTaskFactory.CreateTask("ISIRTaskUpdater"))

        self.in_ctrl_trigger_port  = self.addCreateInputPort("ctrl_trigger",   "int", True)
        self.out_task_updated_port = self.addCreateOutputPort("tasks_updated", "int")


    def register(self, new_task_controller):
        """
        """
        assert( isinstance(new_task_controller, ISIRTaskController) )
        self.task_controllers.append(new_task_controller)

    def remove(self, old_task_controller):
        """
        """
        self.task_controllers.remove(old_task_controller)

    def startHook(self):
        self.task_controllers = []

    def stopHook(self):
        pass

    def updateHook(self):
        tick, tick_ok = self.in_ctrl_trigger_port.read()
        if tick_ok:
            for t_ctrl in self.task_controllers:    #TODO: should be parallelized
                t_ctrl.update(tick)

            self.out_task_updated_port.write(tick)  #all task updates done





