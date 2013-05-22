#!/xde

"""
"""

import task_controller

import lgsm

import numpy as np

from scipy.interpolate import piecewise_polynomial_interpolate as ppi


################################################################################
#
# Miscalleneous Functions for Walking
#
################################################################################

def traj2zmppoints(comtraj, step_length, step_side, left_start, right_start, start_foot):
    """Generate a set of points to locate the feet position around a trajectory
    of the Center of Mass.

    :param comtraj: list of 3 parameters: [x_traj, y_traj, angular_traj]

    :param step_length: the distance done with one step in meter
    :param step_side: the distance between the feet and the CoM trajectory

    :param left_start: left foot pos [x_pos, y_pos, angular_pos]
    :param right_start: right foot pos [x_pos, y_pos, angular_pos]

    :param start_foot: 'left'/'l' or 'right'/'r'
    :type start_foot: string

    :return: a list of points which represent the feet location on floor

    """
    left_start  = np.asarray(left_start)
    right_start = np.asarray(right_start)
    point = []

    if   start_foot == 'left' :
        point.extend([left_start, right_start])
    elif start_foot == 'right':
        point.extend([right_start, left_start])
    else:
        raise ValueError
    next_foot = start_foot

    sum_distance = 0.
    for i in np.arange(len(comtraj)-1):
        sum_distance += np.linalg.norm(comtraj[i+1][0:2]-comtraj[i][0:2])

        if sum_distance > step_length:
            angle = comtraj[i][2]
            ecart = step_side*np.array([-np.sin(angle), np.cos(angle), 0])
            if next_foot is 'right':
                ecart = -ecart
            point.append(comtraj[i] + ecart)
            sum_distance = 0.
            next_foot = 'right' if next_foot == 'left' else 'left'

    # just to get the 2 last footsteps
    angle = comtraj[-1][2]
    ecart = step_side*np.array([-np.sin(angle), np.cos(angle), 0])
    if next_foot == 'left':
        point.extend([comtraj[-1] + ecart, comtraj[-1] - ecart])
    else:
        point.extend([comtraj[-1] - ecart, comtraj[-1] + ecart])
    return point



def zmppoints2zmptraj(point, step_time, dt):
    """Get the Zero Moment Point trajectory from feet location.

    :param point: the list of the feet location
    :param step_time: the time between 2 steps
    :param dt: dt of simulation

    :return: the ZMP traj [x_traj, y_traj]

    """
    gab2 = np.ones((round(step_time/(dt*2.) ), 1))
    gab  = np.ones((round(step_time/dt),       1))

    start = np.dot(gab2, point[0][0:2].reshape(1, 2))
    mid   = [np.dot(gab, p[0:2].reshape(1, 2)) for p in point[1:-1]]
    end   = (point[-2][0:2] + point[-1][0:2])/2.
    traj  = np.vstack( [start]+mid+[end] )

    return traj



def zmppoints2foottraj(point, step_time, ratio, step_height, dt, H_0_planeXY): #cdof, R0):
    """Compute the trajectory of the feet.

    :param point: the list of the feet location
    :param step_time: the time between 2 steps
    :param ratio: ratio between sigle support phase time and step_time
    :param step_height: the max distance between the foot and the floor
    :param dt: dt of simulation
    :param H_0_plane_XY: the transformation matrix from 0 to the floor

    :return: a list with all step trajectories [(pos_i, vel_i, acc_i)]

    """
    def get_bounded_angles(p0, p1):
        #WARNING: do this trick to get the shortest path:
        a0, a1 = (p0[2])%(2*np.pi), (p1[2])%(2*np.pi)
        diff = abs(a1 - a0)
        if   abs(a1+2*np.pi - a0) <diff:
            a1 += 2*np.pi
        elif abs(a1-2*np.pi - a0) <diff:
            a1 -= 2*np.pi
        return a0, a1

    foot_traj = []

    Adj_0_planeXY = lgsm.Displacement(lgsm.vector(0,0,0), H_0_planeXY.getRotation()).adjoint()

    xin   = [0, step_time*ratio]
    xin_Z = [0, step_time*ratio/2., step_time*ratio]
    xout  = np.arange(0, step_time*ratio+dt, dt)
    yin_Z = [[0, 0, 0], [step_height, 0], [0, 0, 0]]

    for i in np.arange(len(point)-2):
        
        a_start, a_end = get_bounded_angles(point[i], point[i+2])
        yin_X = [[point[i][0], 0, 0], [point[i+2][0], 0, 0]]
        yin_Y = [[point[i][1], 0, 0], [point[i+2][1], 0, 0]]
        yin_A = [[a_start,     0, 0], [a_end,         0, 0]]

        res_X = (ppi(xin  , yin_X, xout), ppi(xin  , yin_X, xout, der=1), ppi(xin  , yin_X, xout, der=2))
        res_Y = (ppi(xin  , yin_Y, xout), ppi(xin  , yin_Y, xout, der=1), ppi(xin  , yin_Y, xout, der=2))
        res_Z = (ppi(xin_Z, yin_Z, xout), ppi(xin_Z, yin_Z, xout, der=1), ppi(xin_Z, yin_Z, xout, der=2))
        res_A = (ppi(xin  , yin_A, xout), ppi(xin  , yin_A, xout, der=1), ppi(xin  , yin_A, xout, der=2))

        traj_foot_i = []

        for j in np.arange(len(xout)):

            pos_j = lgsm.Displacement(lgsm.vector(res_X[0][j], res_Y[0][j], res_Z[0][j]), lgsm.Quaternion(np.cos(res_A[0][j]/2.), 0, 0, np.sin(res_A[0][j]/2.) ))
            vel_j = lgsm.Twist( lgsm.vector(0, 0, res_A[1][j], res_X[1][j], res_Y[1][j], res_Z[1][j]) )
            acc_j = lgsm.Twist( lgsm.vector(0, 0, res_A[2][j], res_X[2][j], res_Y[2][j], res_Z[2][j]) )

            traj_foot_i.append( (H_0_planeXY * pos_j, Adj_0_planeXY * vel_j, Adj_0_planeXY * acc_j) )

        foot_traj.append(traj_foot_i)

    return foot_traj




################################################################################
################################################################################
################################################################################
from core import ISIRTaskController

class FootTrajController(ISIRTaskController):
    """
    """
    def __init__(self, lf_ctrl, rf_ctrl, lf_contacts, rf_contacts, ftraj, step_time, step_ratio, dt, start_foot, contact_as_objective):
        """
        """
        self.lf_ctrl     = lf_ctrl
        self.rf_ctrl     = rf_ctrl
        self.lf_contacts = lf_contacts
        self.rf_contacts = rf_contacts
        
        self.contact_as_objective = contact_as_objective

        self.foot_traj  = ftraj
        self.step_time  = step_time
        self.step_ratio = step_ratio
        self.dt         = dt
        self.t          = 0.

        self.sequence     = (np.arange(len(ftraj)+1) + .5)*step_time
        self.ratio_time   = self.step_time*(1 - self.step_ratio)/2.
        self.num_step     = 0
        self.current_foot = 'left' if start_foot=='right' else 'right' #inverse because first foot is not start foot


    def update(self, tick):
        """
        """
        if len(self.sequence) and self.num_step < len(self.sequence):

            self.t += self.dt
            seq_time = self.sequence[self.num_step]

            if self.t >= seq_time + self.ratio_time:
                self.start_next_foot_trajectory()
                print "START traj and deactivate contact of FOOT", self.current_foot

            elif self.t >= seq_time - self.ratio_time:
                print "reactivate contact of FOOT", self.current_foot
                self.stop_current_foot_trajectory()


    def stop_current_foot_trajectory(self):
        """
        """
        if   self.current_foot == 'left' :
            contacts = self.lf_contacts

        elif self.current_foot == 'right':
            contacts = self.rf_contacts

        if self.contact_as_objective is True:
            for c in contacts:
                c.activateAsObjective()
        else:
            for c in contacts:
                c.activateAsConstraint()


    def start_next_foot_trajectory(self):
        """
        """
        self.current_foot = 'left' if self.current_foot=='right' else 'right'
        
        if self.num_step < len(self.foot_traj):
            if   self.current_foot == 'left' :
                contacts  = self.lf_contacts
                foot_ctrl = self.lf_ctrl

            elif self.current_foot == 'right':
                contacts  = self.rf_contacts
                foot_ctrl = self.rf_ctrl

            foot_ctrl.set_new_trajectory( self.foot_traj[self.num_step] )

            for c in contacts:
                c.deactivate()

        self.num_step += 1



################################################################################
################################################################################
################################################################################
class WalkingTask(object):
    """
    """
    def __init__(self, ctrl, dt, H_0_planeXY, lfoot_name, H_lfoot_sole, lf_contacts, rfoot_name, H_rfoot_sole, rf_contacts, com_dofs="XY", weight=1.0, contact_as_objective=False, prefix="walking."):
        """
        """
        self.ctrl = ctrl
        self.dm   = ctrl.dynamic_model

        self.dt = dt

        self.H_0_planeXY = H_0_planeXY
        self.H_planeXY_0 = H_0_planeXY.inverse()

        self.lfoot_index    = self.dm.getSegmentIndex(lfoot_name)
        self.rfoot_index    = self.dm.getSegmentIndex(rfoot_name)
        self.H_lfoot_sole   = H_lfoot_sole
        self.H_rfoot_sole   = H_rfoot_sole
        self.lfoot_contacts = lf_contacts
        self.rfoot_contacts = rf_contacts

        self.contact_as_objective = contact_as_objective

        self.com_task   = ctrl.createCoMTask(prefix+"com", com_dofs, weight, kp=0.) #, kd=0.
        
        H_0_lfs = self.dm.getSegmentPosition(self.lfoot_index) * self.H_lfoot_sole
        H_0_rfs = self.dm.getSegmentPosition(self.rfoot_index) * self.H_rfoot_sole
        self.lfoot_task = ctrl.createFrameTask(prefix+"left_foot" , lfoot_name, H_lfoot_sole, "RXYZ", weight, kp=150., kd=None, pos_des=H_0_lfs)
        self.rfoot_task = ctrl.createFrameTask(prefix+"right_foot", rfoot_name, H_rfoot_sole, "RXYZ", weight, kp=150., kd=None, pos_des=H_0_rfs)

        self.com_ctrl   = None
        self.feet_ctrl  = None
        self.lfoot_ctrl = task_controller.TrajectoryTracking(self.lfoot_task, [])
        self.rfoot_ctrl = task_controller.TrajectoryTracking(self.rfoot_task, [])

        self.ctrl.task_updater.register( self.lfoot_ctrl )
        self.ctrl.task_updater.register( self.rfoot_ctrl )

        self.set_zmp_control_parameters()
        self.set_step_parameters()


    def set_zmp_control_parameters(self, QonR=1e-6, horizon=1.6, stride=3, gravity=9.81):
        """
        """
        self.QonR    = QonR
        self.horizon = horizon
        self.stride  = stride
        self.gravity = gravity

    def set_step_parameters(self, length=.1, side=.05, height=.01, time=1, ratio=.9, start_foot="left"):
        """
        """
        self.length     = length
        self.side       = side
        self.height     = height
        self.step_time  = time
        self.ratio      = ratio
        self.start_foot = start_foot

    def get_center_of_feet_in_XY(self):
        """
        """
        plf_XY = self.get_lfoot_pose_in_XY()[0:2]
        prf_XY = self.get_rfoot_pose_in_XY()[0:2]
        return (plf_XY + prf_XY)/2.

    def get_lfoot_pose_in_XY(self):
        """
        """
        H_0_lfs = self.dm.getSegmentPosition(self.lfoot_index) * self.H_lfoot_sole
        return self.get_pose_in_XY(H_0_lfs)

    def get_rfoot_pose_in_XY(self):
        """
        """
        H_0_rfs = self.dm.getSegmentPosition(self.rfoot_index) * self.H_rfoot_sole
        return self.get_pose_in_XY(H_0_rfs)

    def get_pose_in_XY(self, H_0_pos):
        """
        """
        H_XY_pos = self.H_planeXY_0 * H_0_pos
        R = H_XY_pos.getRotation()
        angle    = 2. * np.arctan2( R.z, R.w )
        return np.array( [H_XY_pos.x, H_XY_pos.y, angle] )


    def stayIdle(self):
        """
        """
        mid_feet = self.get_center_of_feet_in_XY()

        if self.com_ctrl is not None:
            self.ctrl.task_updater.remove( self.com_ctrl )

        self.com_ctrl = task_controller.ZMPController( self.com_task, self.dm, [mid_feet], QonR=self.QonR, horizon=self.horizon, dt=self.dt, H_0_planeXY=self.H_0_planeXY, stride=self.stride, gravity=self.gravity)
        self.ctrl.task_updater.register( self.com_ctrl )


    def goTo(self, pos_in_XY, angle=None, search_path_tolerance=1e-2):
        """
        """
        start = self.get_center_of_feet_in_XY()
        end   = np.asarray(pos_in_XY)
        direction_vector = (end - start)
        path_length = np.linalg.norm(direction_vector)
        if angle is None:
            angle = np.arctan2(direction_vector[1], direction_vector[0])

        N = int(path_length/search_path_tolerance)
        traj = np.array([np.linspace(start[0], end[0], N), np.linspace(start[1], end[1], N), angle*np.ones(N)]).T

        l_start, r_start = self.get_lfoot_pose_in_XY(), self.get_rfoot_pose_in_XY()

        points  = traj2zmppoints(traj, self.length, self.side, l_start, r_start, self.start_foot)
        zmp_ref = zmppoints2zmptraj(points, self.step_time, self.dt)
        ftraj   = zmppoints2foottraj(points, self.step_time, self.ratio, self.height, self.dt, self.H_0_planeXY)

        if self.com_ctrl is not None:
            self.ctrl.task_updater.remove( self.com_ctrl )
        if self.feet_ctrl is not None:
            self.ctrl.task_updater.remove( self.feet_ctrl )

        self.com_ctrl = task_controller.ZMPController( self.com_task, self.dm, zmp_ref, QonR=self.QonR, horizon=self.horizon, dt=self.dt, H_0_planeXY=self.H_0_planeXY, stride=self.stride, gravity=self.gravity)
        self.ctrl.task_updater.register( self.com_ctrl )

        self.feet_ctrl = FootTrajController(self.lfoot_ctrl, self.rfoot_ctrl, self.lfoot_contacts, self.rfoot_contacts, ftraj, self.step_time, self.ratio, self.dt, self.start_foot, self.contact_as_objective)
        self.ctrl.task_updater.register( self.feet_ctrl )

        return zmp_ref




