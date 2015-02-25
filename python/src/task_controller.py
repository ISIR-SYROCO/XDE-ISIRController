#!/xde

""" Module to modify the desired values of tasks.
"""

import swig_isir_controller as sic

import lgsm

class TrajectoryTracking(object):
    """ It modifies the desired values of a task to follow trajectory.
    """

    def __init__(self, task, trajectory=None, expressed_in_world=False):
        """
        :param task: The task to be controlled, meaning the part of the robot that should follow the trajectory
        :type  task: :class:`~core.ISIRTask`
        :param list trajectory: The trajectory to follow, a list ``[(pos1,vel1,acc1), (pos2,vel2,acc2), ..., (posN,velN,accN)]`` for each time step

        """
        if trajectory is None:
            trajectory = []

        self.task = task
        self.set_new_trajectory(trajectory)
        self.expressed_in_world = expressed_in_world

        # check task type
        if task._targetState is None:
            raise ValueError("Cannot control trajectory of task '"+task.getName()+"' that has no target State/Feature")
        elif isinstance(task._targetState, sic.FullTargetState) or isinstance(task._targetState, sic.PartialTargetState):
            if task.getTaskType()   == sic.ACCELERATIONTASK:
                self._doUpdateTask_ = self._updateJointAccelerationTask_
            elif task.getTaskType() == sic.TORQUETASK:
                self._doUpdateTask_ = self._updateTorqueTask_
            else:
                raise ValueError("Control impossible in joint space for FORCETASK or UNKNOWNTASK")
        elif isinstance(task._targetState, sic.TargetFrame):
            if task.getTaskType()   == sic.ACCELERATIONTASK:
                self._doUpdateTask_ = self._updateCartesianAccelerationTask_
            elif task.getTaskType() == sic.FORCETASK:
                self._doUpdateTask_ = self._updateForceTask_
            else:
                raise ValueError("Control impossible in cartesian space for TORQUETASK or UNKNOWNTASK")
        else:
            raise ValueError("Trajectory control of the target state (type:"+type(task._targetState)+") of task '"+task.getName()+"'")


    def set_new_trajectory(self, new_traj):
        """ Register a new trajectory.

        :param list new_traj: The new trajectory to follow, a list ``[(pos1,vel1,acc1), (pos2,vel2,acc2), ..., (posN,velN,accN)]`` for each time step

        The internal counter is reset to 0 and the max counter is ``len(new_traj)``.

        """
        self.trajectory  = new_traj
        self.counter     = 0
        self.max_counter = len(self.trajectory)


    def update(self):
        """ Update the desired value of the registered task to the corresponding element of the trajectory.

        An internal counter selects the corresponding desired value of the trajctory for the task.
        If the last value of the trajectory is reached, the last desired value is considered (it remains registered).

        """
        if self.counter < self.max_counter:
            self._doUpdateTask_()
            self.counter += 1

    def _updateCartesianAccelerationTask_(self):
        pos_des, vel_des, acc_des = self.trajectory[self.counter]
        if self.expressed_in_world:
            Adj_frame_0 = lgsm.Displacementd( [0,0,0] + pos_des.getRotation().inverse().tolist()).adjoint()
            vel_des = Adj_frame_0 * vel_des
            acc_des = Adj_frame_0 * acc_des
        self.task.setPosition(pos_des)
        self.task.setVelocity(vel_des)
        self.task.setAcceleration(acc_des)

    def _updateJointAccelerationTask_(self):
        q_des, qdot_des, qddot_des = self.trajectory[self.counter]
        self.task.set_q(q_des)
        self.task.set_qdot(qdot_des)
        self.task.set_qddot(qddot_des)

    def _updateTorqueTask_(self):
        tau_des = self.trajectory[self.counter]
        self.task.set_tau(tau_des)

    def _updateForceTask_(self):
        pose_ref, wrench_des = self.trajectory[self.counter]
        if self.expressed_in_world:
            Adj_frame_0 = lgsm.Displacementd( [0,0,0] + pos_des.getRotation().inverse().tolist()).adjoint()
            wrench_des = Adj_frame_0 * wrench_des
        self.task.setPosition(pose_ref)
        self.task.setWrench(wrench_des)


import numpy as np
import lgsm

class ZMPController(object):
    """ Compute the desired trajectory of a CoM task based on the ZMP control through a preview controller.
    
    The computation of the desired value is done with the XDE-ZMPy module.
    
    """

    def __init__(self, comtask, dyn_model, goal, RonQ, horizon, dt, H_0_planeXY, stride=1, gravity=9.81, height=0.0, updatePxPu=True):
        """
        :param comtask: The CoM task of the standing/walking robot to control
        :type  comtask: :class:`~core.ISIRTask`
        :param dyn_model: The dynamic model of the robot to control
        :type  dyn_model: :class:`physicshelper.DynamicModel`
        :param goal: The trajectory to follow as best as possible with the ZMP on the XY plane
        :type  goal: (N,2)-array
        :param double RonQ: the ratio between the tracking of the control vector and the tracking of the state vector
        :param double horizon: The time horizon of prediction (in second)
        :param double dt: The time step of the tracked trajectory
        :param H_0_planeXY: The coordinante of the plane where is projected the ZMP trajctory
        :type  H_0_planeXY: :class:`lgsm.Displacement`
        :param int stride: The controller will condired all (stride)-values for the trajectory; higher stride reduces the matrices size, and the updating computation time
        :param double gravity: The amplitude of the gravity vector
        :param double height: The reference height of the CoM
        :param updatePxPu: Whether to update ZMP matrices, mainly if CoM changes
        :type  updatePxPu: bool or float
        
        If `updatePxPu` is set to:
        
        * False, then the matrices are not updated, they are computed based on the height reference set with `height` argument;
        * True, then the matrices are updated at each time step
        * float (a tolerance), then if the reference height move beyond this `tolerance`, the matrices are updated and the new height becomes the reference

        """
        import zmpy_python
        
        self.comtask = comtask
        self.dm      = dyn_model
        
        self.zmp_ctrl = zmpy_python.ZMPController(horizon, dt, RonQ, stride, gravity, height)
        
        self.zmp_ctrl.setGoal(goal)
        
        self._dt = dt
        self._updatePxPu = updatePxPu
        self._height_ref = height
        
        self.R_0_planeXY = H_0_planeXY.getRotation()
        self.H_planeXY_0 = H_0_planeXY.inverse()
        self.R_planeXY_0 = self.H_planeXY_0.getRotation()

        self._prev_vel = np.array( self.R_planeXY_0 * self.dm.getCoMVelocity() ).flatten()
        self._counter  = 0

        self.comtask.setPosition(lgsm.Displacement())
        self.comtask.setVelocity(lgsm.Twist())
        self.accdes = lgsm.Twist()



    def update(self):
        """ Update the desired value of the registered CoM task to the corresponding element of the trajectory.

        It computes the desired CoM acceleration based on the jerk to control the ZMP position.

        """
        
        pos  = np.array( self.H_planeXY_0 * self.dm.getCoMPosition() ).flatten()
        vel  = np.array( self.R_planeXY_0 * self.dm.getCoMVelocity() ).flatten()
        dvel = (vel - self._prev_vel)/self._dt
        self._prev_vel = vel.copy()
        height = pos[2]
        
        
        if self._updatePxPu in (True, False):
            dVcom_des_XY = self.zmp_ctrl.update(pos[0:2], vel[0:2], dvel[0:2], height, self._updatePxPu)
        else:
            if abs(height - self._height_ref) > self._updatePxPu:
                dVcom_des_XY = self.zmp_ctrl.update(pos[0:2], vel[0:2], dvel[0:2], height, True)
                self._height_ref = height
            else:
                dVcom_des_XY = self.zmp_ctrl.update(pos[0:2], vel[0:2], dvel[0:2], height, False)
        
        
        dVcom_des = self.R_0_planeXY * lgsm.vector(dVcom_des_XY[0], dVcom_des_XY[1], 0)
        
        self.accdes.setLinearVelocity( dVcom_des )

        #self.comtask.setPosition(lgsm.Displacement())  # set in initialization
        #self.comtask.setVelocity(lgsm.Twist())         # set in initialization
        self.comtask.setAcceleration(self.accdes)


