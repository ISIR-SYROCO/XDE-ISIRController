#!/xde

"""
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
        self.max_counter = len(self.trajectory) -1



def get_quadratic_cmd(Px, Pu, QonR, h, x_hat, z_ref):
    """
    """
    cmd_traj = - np.dot( np.linalg.inv(np.dot(Pu.T, Pu) + QonR*np.eye(h)), np.dot(Pu.T, np.dot(Px, x_hat) - z_ref) )
    return cmd_traj


import numpy as np
import lgsm


class ZMPController(ISIRTaskController):
    """
    """
    def __init__(self, comtask, dyn_model, goal, QonR, horizon, dt, H_0_planeXY, stride=1, gravity=9.81):
        """
        """
        self.comtask = comtask
        self.dm      = dyn_model

        self._goal    = np.asarray(goal)
        self._QonR    = QonR
        self._dt      = dt * stride
        self._real_dt = dt
        self._stride  = stride
        self._gravity   = gravity
        self._h       = int(horizon/self._dt)

        self.R_0_planeXY = H_0_planeXY.getRotation()
        self.H_planeXY_0 = H_0_planeXY.inverse()
        self.R_planeXY_0 = self.H_planeXY_0.getRotation()

        self._prev_vel = np.array( self.R_planeXY_0 * self.dm.getCoMVelocity() ).flatten()
        self._counter  = 0

        self.posdes = lgsm.Displacement()
        self.veldes = lgsm.Twist()
        self.accdes = lgsm.Twist()

        self._Px       = np.zeros((self._h, 3))
        self._Pu       = np.zeros((self._h, self._h))

        self._Px[:, 0] = 1
        self._Px[:, 1] = np.arange(1, self._h+1)*self._dt

        self._range_N_dt_2_on_2 = (np.arange(1, self._h+1)*self._dt)**2/2.
        self._temp_Pu           = np.zeros((self._h, self._h))

        for i in np.arange(self._h):
            diag_i = (1 + 3*i + 3*i**2)*self._dt**3/6
            self._temp_Pu[np.arange(i, self._h), np.arange(self._h-i)] = diag_i
        self._ltri_idx = np.tril_indices(self._h, -1)


    def _get_com_hat_and_hong(self):
        """
        """
        pos  = np.array( self.H_planeXY_0 * self.dm.getCoMPosition() ).flatten()
        vel  = np.array( self.R_planeXY_0 * self.dm.getCoMVelocity() ).flatten()
        dvel = (vel - self._prev_vel)/self._real_dt

        self._prev_vel = vel.copy()

        com_hat = np.array([pos, vel, dvel])[:, 0:2]
        hong = pos[2]/self._gravity

        return com_hat, hong


    def _fit_goal_for_horizon(self):
        """
        """
        goal = self._goal[self._counter:self._counter+(self._h*self._stride):self._stride]
        self._counter += 1
        if len(goal) < self._h:
            final_value = self._goal[-1].reshape((1, 2))
            len_gap = self._h - len(goal)
            added_part = np.dot(np.ones((len_gap, 1)), final_value)
            goal = np.vstack([goal, added_part])
        return goal


    def _update_Px_and_Pu(self, hong):
        """
        """
        #self._Px[:,0] = 1                                  #already computed in __init__
        #self._Px[:,1] = np.arange(1, self._h+1)*self._dt   #idem
        self._Px[:, 2] = self._range_N_dt_2_on_2 - hong

        self._Pu[:]               = self._temp_Pu
        self._Pu[self._ltri_idx] -= self._dt*hong


    def update(self, tick):
        """
        """
        com_hat, hong = self._get_com_hat_and_hong()

        zmp_ref = self._fit_goal_for_horizon()
        self._update_Px_and_Pu(hong)

        ddV_com_XY = get_quadratic_cmd(self._Px, self._Pu,  self._QonR, self._h,  com_hat, zmp_ref)

        dVcom_des_XY      = np.zeros(3)
        dVcom_des_XY[0:2] = com_hat[2, :] + ddV_com_XY[0] * self._real_dt

        dVcom_des = self.R_0_planeXY * dVcom_des_XY

        self.accdes.setLinearVelocity( lgsm.vector(dVcom_des) )

        self.comtask.update(self.posdes, self.veldes, self.accdes)


