#!/xde

""" Module to modify the desired values of tasks.
"""

from core import ISIRTaskController


class TrajectoryTracking(ISIRTaskController):
    """ It modifies the desired values of a task to follow trajectory.
    """

    def __init__(self, task, trajectory):
        """
        :param task: The task to be controlled, meaning the part of the robot that should follow the trajectory
        :type  task: class:`~core.ISIRTask`
        :param list trajectory: The trajectory to follow, a list [(pos1,vel1,acc1), (pos2,vel2,acc2), ..., (posN,velN,accN)] for each time step

        """
        ISIRTaskController.__init__(self)
        
        self.task = task
        self.set_new_trajectory(trajectory)

    def update(self, tick):
        """ Update the desired value of the registered task to the corresponding element of the trajectory.

        :param int tick: the current physic agent iteration index (unused here)

        An internal counter selects the corresponding desired value of the trajctory for the task.
        If the last value of the trajectory is reached, the the last desired value is considered (it remains registered)

        """
        if self.counter < self.max_counter:

            traj_des = self.trajectory[self.counter]
            self.task.update(*traj_des)
            self.counter += 1

    def set_new_trajectory(self, new_traj):
        """ Register a new trajectory.

        :param list new_traj: The new trajectory to follow, a list [(pos1,vel1,acc1), (pos2,vel2,acc2), ..., (posN,velN,accN)] for each time step

        The internal counter is reset to 0 and the max counter is len(new_traj).

        """
        self.trajectory  = new_traj
        self.counter     = 0
        self.max_counter = len(self.trajectory)



import numpy as np
import lgsm

class ZMPController(ISIRTaskController):
    """ Compute the desired trajectory of a CoM task based on the ZMP control through a preview controller.
    
    The computation of the desired value is done with the XDE-ZMPy module.

    """
    
    def __init__(self, comtask, dyn_model, goal, RonQ, horizon, dt, H_0_planeXY, stride=1, gravity=9.81, height=0.0, updatePxPu=True, use_swig_zmpy=True):
        """
        :param comtask: The CoM task of the standing/walking robot to control
        :type  comtask: class:`~core.ISIRTask`
        :param dyn_model: The dynamic model of the robot to control
        :type  dyn_model: class:`physicshelper.DynamicModel`
        :param goal: The trajectory to follow as best as possible with the ZMP on the XY plane
        :type  goal: (X,2)-array
        :param double RonQ: the ratio between the tracking of the control vector and the tracking of the state vector
        :param double horizon: The time horizon of prediction (in second)
        :param double dt: The time step of the tracked trajectory
        :param H_0_planeXY: The coordinante of the plane where is projected the ZMP trajctory
        :type  H_0_planeXY: class:`lgsm.Displacement`
        :param int stride: The controller will condired all (stride)-values for the trajectory; higher stride reduces the matrices size, and the updating computation time
        :param double gravity: The amplitude of the gravity vector
        :param double height: The reference height of the CoM
        :param updatePxPu: Whether to update ZMP matrices, mainly if CoM changes
        :type  updatePxPu: (bool or float)
        :param bool use_swig_zmpy: Whether to use a swig (True) or a python (False) computation of the ZMP matrices
        
        If `updatePxPu` is set to:
        
        * `False`, then the matrices are not updated, they are computed based on the height reference set with `height` argument;
        * `True`, then the matrices are updated at each time step
        * `float:tolerance`, then if the reference height move beyond this `tolerance`, then matrices are updated and new height becomes the reference

        """
        import xde_zmpy
        
        self.comtask = comtask
        self.dm      = dyn_model
        
        if use_swig_zmpy:
            self.zmp_ctrl = xde_zmpy.zmpy_swig.ZMPController(horizon, dt, RonQ, stride, gravity, height)
        else:
            self.zmp_ctrl = xde_zmpy.zmpy_python.ZMPController(horizon, dt, RonQ, stride, gravity, height)
        
        self.zmp_ctrl.setGoal(goal)
        
        self._dt = dt
        self._updatePxPu = updatePxPu
        self._height_ref = height
        
        self.R_0_planeXY = H_0_planeXY.getRotation()
        self.H_planeXY_0 = H_0_planeXY.inverse()
        self.R_planeXY_0 = self.H_planeXY_0.getRotation()

        self._prev_vel = np.array( self.R_planeXY_0 * self.dm.getCoMVelocity() ).flatten()
        self._counter  = 0

        self.posdes = lgsm.Displacement()
        self.veldes = lgsm.Twist()
        self.accdes = lgsm.Twist()



    def update(self, tick):
        """ Update the desired value of the registered CoM task to the corresponding element of the trajectory.

        :param int tick: thre current physic agent iteration index (unused here)

        It computes the desired CoM acceleration based on the jerk to control the ZMP position

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

        self.comtask.update(self.posdes, self.veldes, self.accdes)


