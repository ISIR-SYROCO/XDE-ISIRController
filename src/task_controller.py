#!/xde

""" TODO: this module should become C++ code, especially for the ZMP controller, to become more efficient.
"""

from core import ISIRTaskController


class TrajectoryTracking(ISIRTaskController):
    """
    """

    def __init__(self, task, trajectory):
        """
        """
        ISIRTaskController.__init__(self)
        
        self.task = task
        self.set_new_trajectory(trajectory)

    def update(self, tick):
        """
        """
        if self.counter < self.max_counter:

            traj_des = self.trajectory[self.counter]
            self.task.update(*traj_des)
            self.counter += 1

    def set_new_trajectory(self, new_traj):
        """
        """
        self.trajectory  = new_traj
        self.counter     = 0
        self.max_counter = len(self.trajectory)


import numpy as np
import lgsm


class ZMPController(ISIRTaskController):
    """
    """
    
    def __init__(self, comtask, dyn_model, goal, RonQ, horizon, dt, H_0_planeXY, stride=1, gravity=9.81, height=0.0, updatePxPu=True, use_swig_zmpy=True):
        """
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
        """
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


