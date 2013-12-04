#!/usr/bin/env python

""" TODO: this module should become C++ code, especially for the ZMP controller, to become more efficient.
"""



def get_quadratic_cmd(Px, Pu, RonQ, h, x_hat, z_ref):
    """
    """
    cmd_traj = - np.dot( np.linalg.inv(np.dot(Pu.T, Pu) + RonQ*np.eye(h)), np.dot(Pu.T, np.dot(Px, x_hat) - z_ref) )
    return cmd_traj


import numpy as np


class ZMPController(object):
    """
    """
    def __init__(self, horizon, dt, RonQ=1e-6, stride=1, gravity=9.81, height=0.0):
        """
        """

        self._goal    = None
        self._RonQ    = RonQ
        self._dt      = dt * stride
        self._real_dt = dt
        self._stride  = stride
        self._gravity = gravity
        self._N       = int(horizon/self._dt)

        self._counter  = 0

        self._Px       = np.zeros((self._N, 3))
        self._Pu       = np.zeros((self._N, self._N))

        self._Px[:, 0] = 1
        self._Px[:, 1] = np.arange(1, self._N+1)*self._dt

        self._range_N_dt_2_on_2 = (np.arange(1, self._N+1)*self._dt)**2/2.
        self._temp_Pu           = np.zeros((self._N, self._N))

        for i in np.arange(self._N):
            diag_i = (1 + 3*i + 3*i**2)*self._dt**3/6
            self._temp_Pu[np.arange(i, self._N), np.arange(self._N-i)] = diag_i
        self._ltri_idx = np.tril_indices(self._N, 0)    #TODO: 0 or -1 ???

        self._update_Px_and_Pu( height/self._gravity )

    def getPx(self):
        return self._Px

    def getPu(self):
        return self._Pu


    def setGoal(self, new_goal):
        """
        """
        self._goal = np.asarray(new_goal)

    def _fit_goal_for_horizon(self):
        """
        """
        goal = self._goal[self._counter:self._counter+(self._N*self._stride):self._stride]
        self._counter += 1
        if len(goal) < self._N:
            final_value = self._goal[-1].reshape((1, 2))
            len_gap = self._N - len(goal)
            added_part = np.dot(np.ones((len_gap, 1)), final_value)
            goal = np.vstack([goal, added_part])
        return goal


    def _update_Px_and_Pu(self, hong):
        """
        """
        self._Px[:, 2] = self._range_N_dt_2_on_2 - hong

        self._Pu[:]               = self._temp_Pu
        self._Pu[self._ltri_idx] -= self._dt*hong


    def update(self, pos_xy, vel_xy, acc_xy, height=0.0, update_PxPu=True):
        """
        """
        com_hat = np.array([pos_xy, vel_xy, acc_xy])
        hong = height/self._gravity

        zmp_ref = self._fit_goal_for_horizon()
        
        if update_PxPu is True:
            self._update_Px_and_Pu(hong)

        ddV_com_XY = get_quadratic_cmd(self._Px, self._Pu,  self._RonQ, self._N,  com_hat, zmp_ref)

        dVcom_des_XY      = np.zeros(2)
        dVcom_des_XY[0:2] = com_hat[2, :] + ddV_com_XY[0] * self._real_dt

        return dVcom_des_XY


