#!/xde

"""
"""

import task_controller

import lgsm

import numpy as np

from scipy.interpolate import piecewise_polynomial_interpolate as ppi

import time


################################################################################
#
# Miscalleneous Functions for Walking
#
################################################################################

def traj2zmppoints(comtraj, step_length, step_side, left_start, right_start, start_foot):
    """ Generate a set of points to locate the feet position around a trajectory
    of the Center of Mass.

    :param comtraj: list of 3 parameters: [x_traj, y_traj, angular_traj]

    :param step_length: the distance done with one step in meter
    :param step_side: the distance between the feet and the CoM trajectory

    :param left_start:  left foot pos  [x_pos, y_pos, angular_pos]
    :param right_start: right foot pos [x_pos, y_pos, angular_pos]

    :param string start_foot: 'left' or 'right'

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
    """ Get the Zero Moment Point trajectory from feet location.

    :param list point: the list of the feet locations
    :param double step_time: the time between 2 foot steps
    :param double dt: time step of simulation

    :return: the ZMP traj [x_traj, y_traj]

    """
    gab2 = np.ones((round(step_time/(dt*2.) ), 1))
    gab  = np.ones((round(step_time/dt),       1))

    start = np.dot(gab2, point[0][0:2].reshape(1, 2))
    mid   = [np.dot(gab, p[0:2].reshape(1, 2)) for p in point[1:-1]]
    end   = (point[-2][0:2] + point[-1][0:2])/2.
    traj  = np.vstack( [start]+mid+[end] )

    return traj



def get_bounded_angles(p0, p1):
    """ Convert angles to get the shortest angle path.
    
    :param p0: The first point to test in the feet trajectory
    :type  p0: [x_p0, y_p0, a_p0]
    :param p1: The second point to test in the feet trajectory
    :type  p1: [x_p1, y_p1, a_p1]

    :return: [a_p0', a_p1'] such as difference between them is minimal in [-2pi, 2pi]

    """
    #WARNING: do this trick to get the shortest path:
    a0, a1 = (p0[2])%(2*np.pi), (p1[2])%(2*np.pi)
    diff = abs(a1 - a0)
    if   abs(a1+2*np.pi - a0) <diff:
        a1 += 2*np.pi
    elif abs(a1-2*np.pi - a0) <diff:
        a1 -= 2*np.pi
    return a0, a1


def zmppoints2foottraj(points, step_time, ratio, step_height, dt, H_0_planeXY): #cdof, R0):
    """ Compute the trajectory of the feet.

    :param list point: the list of the feet locations
    :param double step_time: the time between 2 steps
    :param double ratio: ratio between single support phase time and complete cycle time
    :param double step_height: the max distance between the foot and the floor
    :param double dt: time step of simulation
    :param H_0_plane_XY: the transformation matrix from 0 to the floor
    :type  H_0_plane_XY: :class:`lgsm.Displacement`

    :return: a list with all step trajectories [[(pos_i, vel_i, acc_i)]]

    """
    foot_traj = []

    Adj_0_planeXY = lgsm.Displacement(lgsm.vector(0,0,0), H_0_planeXY.getRotation()).adjoint()

    xin   = [0, step_time*ratio]
    xin_Z = [0, step_time*ratio/2., step_time*ratio]
    xout  = np.arange(0, step_time*ratio+dt, dt)
    yin_Z = [[0, 0, 0], [step_height, 0], [0, 0, 0]]

    for i in np.arange(len(points)-2):
        
        a_start, a_end = get_bounded_angles(points[i], points[i+2])
        yin_X = [[points[i][0], 0, 0], [points[i+2][0], 0, 0]]
        yin_Y = [[points[i][1], 0, 0], [points[i+2][1], 0, 0]]
        yin_A = [[a_start,      0, 0], [a_end,          0, 0]]

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


def zmppoints2waisttraj(points, step_time, dt, H_0_planeXY):
    """ Compute the trajectory of the waist based on zmp points.
    
    :param list point: the list of the feet locations
    :param double step_time: the time between 2 steps
    :param double dt: time step of simulation
    :param H_0_plane_XY: the transformation matrix from 0 to the floor
    :type  H_0_plane_XY: :class:`lgsm.Displacement`
    
    :return: a list with the whole waist trajectory [(pos_i, vel_i, acc_i)]
    """
    waist_traj = []
    
    Adj_0_planeXY = lgsm.Displacement(lgsm.vector(0,0,0), H_0_planeXY.getRotation()).adjoint()
    
    xin   = [0, step_time]
    xout  = np.arange(0, step_time+dt, dt)
    
    for i in np.arange(len(points)-1):
    
        a_start, a_end = get_bounded_angles(points[i], points[i+1])
        yin_A = [[a_start, 0, 0], [a_end, 0, 0]]

        res_A = (ppi(xin  , yin_A, xout), ppi(xin  , yin_A, xout, der=1), ppi(xin  , yin_A, xout, der=2))
        
        for j in np.arange(len(xout)):

            pos_j = lgsm.Displacement(lgsm.zeros(3), lgsm.Quaternion(np.cos(res_A[0][j]/2.), 0, 0, np.sin(res_A[0][j]/2.) ))
            vel_j = lgsm.Twist( lgsm.vector(0, 0, res_A[1][j], 0,0,0) )
            acc_j = lgsm.Twist( lgsm.vector(0, 0, res_A[2][j], 0,0,0) )

            waist_traj.append( (H_0_planeXY * pos_j, Adj_0_planeXY * vel_j, Adj_0_planeXY * acc_j) )

    return waist_traj

################################################################################
################################################################################
################################################################################
from core import ISIRTaskController

class FootTrajController(ISIRTaskController):
    """ A controller to determine which foot has to be considered linked with the ground and which foot is released.
    """
    def __init__(self, lf_ctrl, rf_ctrl, lf_contacts, rf_contacts, ftraj, step_time, step_ratio, dt, start_foot, contact_as_objective, verbose=False):
        """
        :param lf_ctrl: A controller which will set the trajectory to the left foot
        :type  lf_ctrl: :class:`task_controller.TrajectoryTracking`
        :param rf_ctrl: A controller which will set the trajectory to the right foot
        :type  rf_ctrl: :class:`task_controller.TrajectoryTracking`
        :param list lf_contacts: The list of the contact tasks related to the left foot
        :param list rf_contacts: The list of the contact tasks related to the right foot
        :param list ftraj: a list with all step trajectories [[(pos_i, vel_i, acc_i)]], generally returned by :func:`zmppoints2foottraj`
        :param double step_time: the time between 2 steps
        :param double step_ratio: ratio between single support phase time and complete cycle time
        :param double dt: time step of simulation
        :param string start_foot: 'left'/'l' or 'right'/'r'
        :param bool contact_as_objective: Whether to consider contacts as objective (True) or constraint (False) when they are re-activated
        :param bool verbose: Whether to print information on FootTrajController evolution

        """
        self.lf_ctrl     = lf_ctrl
        self.rf_ctrl     = rf_ctrl
        self.lf_contacts = lf_contacts
        self.rf_contacts = rf_contacts
        
        self.contact_as_objective = contact_as_objective
        self.verbose              = verbose

        self.foot_traj  = ftraj
        self.step_time  = step_time
        self.step_ratio = step_ratio
        self.dt         = dt
        self.t          = 0.

        self.sequence     = (np.arange(len(ftraj)+1) + .5)*step_time
        self.ratio_time   = self.step_time*(1 - self.step_ratio)/2.
        self.num_step     = 0
        self.current_foot = 'left' if start_foot=='right' else 'right' #inverse because first foot is not start foot

        self.status_is_walking           = True     # when initialized, it starts to walk
        self.status_is_on_double_support = True
        self.status_is_on_simple_support = False


    def update(self, tick):
        """ Update the desired value of the registered feet tasks to the corresponding element of the trajectory.

        :param int tick: thre current physic agent iteration index (unused here)

        Here, it looks if the current step is still in the sequence; if true,
        it check the internal time (t) of the controller:

        * if (t) is lower than current step time + ratio_time, it activates next foot and start its motion by deactivating contact tasks
        * if (t) is greater than current step time - ratio_time, it deactivates current foot and reactivate contact tasks

        """
        if len(self.sequence) and self.num_step < len(self.sequence):

            self.t += self.dt
            seq_time = self.sequence[self.num_step]

            if self.t >= seq_time + self.ratio_time:
                self.start_next_foot_trajectory()
                if self.verbose:
                    print "START traj and deactivate contact of FOOT", self.current_foot

            elif self.t >= seq_time - self.ratio_time:
                if self.verbose:
                    print "reactivate contact of FOOT", self.current_foot
                self.stop_current_foot_trajectory()

        else:
            self.status_is_walking = False


    def stop_current_foot_trajectory(self):
        """ Stop the current foot motion.

        It reactivates the contact tasks related to the corresponding foot.
        They are considered as either objectives or constraints, depending on argument in constructor.
        """
        self.status_is_on_double_support = True
        self.status_is_on_simple_support = False

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
        """ Start the motion of the next foot in the walking sequence.

        It deactivates the contact tasks related to the corresponding foot,
        and activates its foot control with a new trajectory.

        """
        self.status_is_on_double_support = False
        self.status_is_on_simple_support = self.current_foot

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


    def is_walking(self):
        """ Return True if the robot is still walking, False otherwise.
        """
        return self.status_is_walking

    def is_on_double_support(self):
        """ Return True if the robot has the 2 feet on the ground, False otherwise.
        """
        return self.status_is_on_double_support

    def is_on_simple_support(self):
        """ Return 'left' or 'right' if the corresponding foot is the only one on the ground, False otherwise.
        """
        return self.status_is_on_simple_support



################################################################################
################################################################################
################################################################################
class WalkingActivity(object):
    """ Class that creates and parameterizes the tasks needed for genrating the walking activity.

    Here, the walk is considered as an **activity**, meaning that it regroups many sub-atomic-tasks which are:

    * the CoM task, controlled through the preview control of the ZMP,
    * the feet tasks,
    * the root/pelvis task, to maintain the upper part of the robot upstraight.

    """

    def __init__(self, ctrl, dt, lfoot_name, H_lfoot_sole, lf_contacts, rfoot_name, H_rfoot_sole, rf_contacts, waist_name, H_waist_front, waist_position, H_0_planeXY=None, horizontal_dofs="XY", vertical_dof="Z", weight=1.0, contact_as_objective=False, prefix="walking."):
        """
        :param ctrl: The ISIRController instance that will create the tasks and control them
        :type  ctrl: :class:`core.ISIRCtrl`
        :param double dt: time step of simulation
        :param string lfoot_name: The name of the left foot segment
        :param H_lfoot_sole: The displacement from the left foot segment frame to the left sole reference frame
        :type  H_lfoot_sole: :class:`lgsm.Displacement`
        :param list lf_contacts: A list of contact tasks representing the left foot contact points
        :param string rfoot_name: The name of the left foot segment
        :param H_rfoot_sole: The displacement from the right foot segment frame to the right sole reference frame
        :type  H_rfoot_sole: :class:`lgsm.Displacement`
        :param list rf_contacts: A list of contact tasks representing the right foot contact points
        :param string waist_name: The name of the waist/pelvis segment
        :param H_waist_front: The displacement from the waist segment frame to the waist reference frame
        :type  H_waist_front: :class:`lgsm.Displacement`
        :param waist_position: The desired pose for the waist reference frame, only the `vertical_dof` is controlled (generally 'Z')
        :type  waist_position: :class:`lgsm.Displacement`
        :param H_0_planeXY: The displacement from the ground to the origin of the XY-plane where are projected all the trajectories, CoM, ZMP, feet, waist, etc...
        :type  H_0_planeXY: :class:`lgsm.Displacement`
        :param string horizontal_dofs: The controlled axis on the horizontal plane relative to the ground ("XY", "XZ" or "YZ")
        :param string vertical_dof: The controlled vertical axis relative to the ground ("X", "Y" or "Z")
        :param double weight: The reference weight for all the sub-tasks created by this activity
        :param bool contact_as_objective: Whether to consider feet contacts as objective (True) or constraint (False) when they are re-activated
        :param string prefix: Prefix string added to the name of each sub-task name
        
        The left/right sole reference frames defined with `H_lfoot_sole` and `H_rfoot_sole` are the frame controlled
        by the feet tasks, and they represent the feet centers.
        The same is done for the waist reference frame defined by `H_waist_front` controlled by
        the waist task.
        All these frames should be aligned as follows:

        * The Z-axis must be pointing upwards
        * The X-axis must be pointing forwards

        When the CoM trajectory is given with an angle, it corresponds to the angle between
        the X-axis of the plane defiend by `H_0_planeXY` and the X-axis of the feet/waist.
        
        """
        self.ctrl = ctrl
        self.dm   = ctrl.dynamic_model

        self.dt = dt

        if H_0_planeXY is None:
            H_0_planeXY = lgsm.Displacement()
        self.H_0_planeXY = H_0_planeXY
        self.H_planeXY_0 = H_0_planeXY.inverse()

        self.lfoot_index    = self.dm.getSegmentIndex(lfoot_name)
        self.rfoot_index    = self.dm.getSegmentIndex(rfoot_name)
        self.waist_index    = self.dm.getSegmentIndex(waist_name)
        self.H_lfoot_sole   = H_lfoot_sole
        self.H_rfoot_sole   = H_rfoot_sole
        self.H_waist_front  = H_waist_front
        self.lfoot_contacts = lf_contacts
        self.rfoot_contacts = rf_contacts

        self.contact_as_objective = contact_as_objective

        # Creation of all the sub-atomic-tasks
        H_0_lfs = self.dm.getSegmentPosition(self.lfoot_index) * self.H_lfoot_sole
        H_0_rfs = self.dm.getSegmentPosition(self.rfoot_index) * self.H_rfoot_sole
        self.lfoot_task = ctrl.createFrameTask(prefix+"left_foot" , lfoot_name, H_lfoot_sole, "RXYZ", weight, kp=150., kd=None, pos_des=H_0_lfs)
        self.rfoot_task = ctrl.createFrameTask(prefix+"right_foot", rfoot_name, H_rfoot_sole, "RXYZ", weight, kp=150., kd=None, pos_des=H_0_rfs)
        self.com_task   = ctrl.createCoMTask(  prefix+"com", horizontal_dofs, weight, kp=0.) #, kd=0.

        self.waist_rot_task = ctrl.createFrameTask(prefix+"waist_rotation", waist_name, H_waist_front, "R"         , weight, kp=9., pos_des=waist_position)
        self.waist_alt_task = ctrl.createFrameTask(prefix+"waist_altitude", waist_name, H_waist_front, vertical_dof, weight, kp=9., pos_des=waist_position)

        # Creation of the sub-tasks controllers, to set the feet/waist/CoM trajectories
        self.com_ctrl   = None
        self.feet_ctrl  = None
        self.lfoot_ctrl = task_controller.TrajectoryTracking(self.lfoot_task, [])
        self.rfoot_ctrl = task_controller.TrajectoryTracking(self.rfoot_task, [])
        self.waist_rot_ctrl = task_controller.TrajectoryTracking(self.waist_rot_task, [])
        self.waist_alt_ctrl = task_controller.TrajectoryTracking(self.waist_alt_task, [])

        self.ctrl.task_updater.register( self.lfoot_ctrl )
        self.ctrl.task_updater.register( self.rfoot_ctrl )
        self.ctrl.task_updater.register( self.waist_rot_ctrl )
        self.ctrl.task_updater.register( self.waist_alt_ctrl )

        self.set_zmp_control_parameters()
        self.set_step_parameters()


    def is_balancing(self):
        """ Return True if the robot is balacing, meaning that the control of the CoM is activated, False otherwise.
        """
        if self.com_ctrl is None:
            return False
        else:
            return True

    def is_walking(self):
        """ Return True if the robot is walking, meaning if it remains some footsteps to reach the desired destination, False otherwise (no step to do or arrived at destiation).
        
        It calls :meth:`FootTrajController.is_walking`
        """
        if self.feet_ctrl is None:
            return False
        else:
            return self.feet_ctrl.is_walking()

    def is_on_double_support(self):
        """ Return True if robot is in the double support phase, False otherwise.
        
        It calls :meth:`FootTrajController.is_on_double_support`
        """
        if self.feet_ctrl is None:
            return True # assume that if no feet control, then it is on double support
        else:
            return self.feet_ctrl.is_on_double_support()

    def is_on_simple_support(self):
        """ Return True if robot is in the simple support phase, False otherwise.
        
        It calls :meth:`FootTrajController.is_on_simple_support`
        """
        if self.feet_ctrl is None:
            return False
        else:
            return self.feet_ctrl.is_on_simple_support()

    def setTasksWeight(self, weight):
        """ Set the same weight for every sub-tasks.
        
        :param double weight: The new weight for all sub-tasks
        
        If one wants to change the weight of a sub-task in particular, get the
        corresponding task instance:
        
        * `self.lfoot_task`
        * `self.rfoot_task`
        * `self.com_task`
        * `self.waist_rot_task`
        * `self.waist_alt_task`
        
        and change the weight with the :meth:`core.ISIRTask.setWeight`
        """
        for t in [self.lfoot_task, self.rfoot_task, self.com_task, self.waist_rot_task, self.waist_alt_task]:
            t.setWeight(weight)

    def set_waist_altitude(self, altitude):
        """ Set the reference altitude of the waist altitude task.
        
        :param double altitude: The new altitude in meter
        
        """
        if isinstance(altitude, lgsm.Displacement):
            altitude = [[altitude]]
        self.waist_alt_ctrl.set_new_trajectory( altitude )

    def set_waist_orientation(self, orientation):
        """ Set the reference orientation of the waist altitude task.
        
        :param orientation: The new reference orientation
        :type  orientation: :class:`lgsm.Displacement`
        
        """
        if isinstance(orientation, lgsm.Displacement):
            orientation = [[orientation]]
        self.waist_rot_ctrl.set_new_trajectory( orientation )

    def set_zmp_control_parameters(self, RonQ=1e-6, horizon=1.6, stride=3, gravity=9.81, height_ref=0.0, updatePxPu=True, use_swig_zmpy=True):
        """ Set the parameters for the ZMP control.
        
        :param double RonQ: the ratio between the tracking of the control vector and the tracking of the state vector
        :param double horizon: The time horizon of prediction (in second)
        :param int stride: The controller will condired all (stride)-values for the trajectory; higher stride reduces the matrices size, and the updating computation time
        :param double gravity: The amplitude of the gravity vector
        :param double height_ref: The reference CoM height, to compute the ZMP preview control matrices, if they are not updated at each time step
        :param bool updatePxPu: Whether to update ZMP matrices, mainly if CoM changes (see below)
        :param bool use_swig_zmpy: Whether to use a swig (True) or a python (False) computation of the ZMP matrices
        
        If `updatePxPu` is set to:
        
        * `False`, then the matrices are not updated, they are computed based on the height reference set with `height_ref` argument;
        * `True`, then the matrices are updated at each time step
        * `float:tolerance`, then if the reference height move beyond this `tolerance`, then matrices are updated and new height becomes the reference
        
        """
        self.RonQ          = RonQ
        self.horizon       = horizon
        self.stride        = stride
        self.gravity       = gravity
        self.height_ref    = height_ref
        self.updatePxPu    = updatePxPu
        self.use_swig_zmpy = use_swig_zmpy

    def set_step_parameters(self, length=.1, side=.05, height=.01, time=1, ratio=.9, start_foot="left"):
        """ Set the parameters for the footsteps, to parameterize the feet trajectories.
        
        :param double length: The relative distance between the feet along the trajectory when a step is performed
        :param double side: The side distance of the footsteps relative to the "middle line" of the CoM trajectory
        :param double height: The maximum altitude reached by the feet when moving, relative to the planeXY origin
        :param double time: The average time between two foot step; the cycle time should be 2*time
        :param double ratio: The time ration simple/double support phases
        :param string start_foot: The starting foot when the walk activity starts ('left' or 'right')
        
        .. todo::
           
           The starting foot should not be set all the time, if a foot a backwards towards the walking direction,it should always start the walk activity. No?
        
        """
        self.length     = length
        self.side       = side
        self.height     = height
        self.step_time  = time
        self.ratio      = ratio
        self.start_foot = start_foot

    def get_center_of_feet_in_XY(self):
        """ Get the average position of the feet in the planeXY.
        
        :return: `[x_pos, y_pos, a_pos]` which are the average positions and angle between the two
        feet reference frames in the planeXY.
        
        """
        plf_XY = self.get_lfoot_pose_in_XY()[0:2]
        prf_XY = self.get_rfoot_pose_in_XY()[0:2]
        return (plf_XY + prf_XY)/2.

    def get_lfoot_pose_in_XY(self):
        """ Get the left foot pose in the planeXY.
        
        :return: `[x_pos, y_pos, a_pos]` which are the positions and angle of the left foot reference frame in the planeXY.
        
        """
        H_0_lfs = self.dm.getSegmentPosition(self.lfoot_index) * self.H_lfoot_sole
        return self.get_pose_in_XY(H_0_lfs)

    def get_rfoot_pose_in_XY(self):
        """ Get the right foot pose in the planeXY.
        
        :return: `[x_pos, y_pos, a_pos]` which are the positions and angle of the right foot reference frame in the planeXY.
        
        """
        H_0_rfs = self.dm.getSegmentPosition(self.rfoot_index) * self.H_rfoot_sole
        return self.get_pose_in_XY(H_0_rfs)

    def get_pose_in_XY(self, H_0_pos):
        """ Convert a pose relative to the ground into a pose in the planeXY.
        
        :param H_0_pos: The pose relative to the ground to convert.
        :type  H_0_pos: :class:`lgsm.Displacement`
        :return: `[x_pos, y_pos, a_pos]` which are the positions and angle in the planeXY.
        
        """
        H_XY_pos = self.H_planeXY_0 * H_0_pos
        R        = H_XY_pos.getRotation()
        angle    = 2. * np.arctan2( R.z, R.w )
        return np.array( [H_XY_pos.x, H_XY_pos.y, angle] )


    def stayIdle(self, com_position=None):
        """ Generate a behavior where the robot stay idle.
        
        :param com_position: A reference position for the com when the robot is static.
        :type  com_position: None or (x_com, y_com)
        
        If `com_position` is None, then the position returned by :meth:`get_center_of_feet_in_XY` is used.
        
        """
        if com_position is None:
            com_position = self.get_center_of_feet_in_XY()

        if self.com_ctrl is not None:
            self.ctrl.task_updater.remove( self.com_ctrl )

        self.com_ctrl = task_controller.ZMPController( self.com_task, self.dm, [com_position], self.RonQ, self.horizon, self.dt, self.H_0_planeXY, self.stride, self.gravity, self.height_ref, self.updatePxPu, self.use_swig_zmpy)
        self.ctrl.task_updater.register( self.com_ctrl )


    def goTo(self, pos_in_XY, relative_pos=False, angle=None, search_path_tolerance=1e-2, verbose=False):
        """ Generate a walking activity where the robot has to reach a desired destination.
        
        :param pos_in_XY: The desired final position expressed in the planeXY.
        :type  pos_in_XY: (x_pos, y_pos)
        :param bool relative_pos: If True, the `pos_in_XY` is considered relatively to the current robot position (returned by :meth:`get_center_of_feet_in_XY`), if False it is considered relatively to the planeXY
        :param double angle: The final angle/orientation of the robot expressed in the planeXY
        :param double search_path_tolerance: The distance for the trajectory discretization
        :param bool verbose: If one wants to get the log information sent by the :class:`FootTrajController`
        
        This function create a discretized straight line from the current position to the desired destination, and a constant angle during the whole motion.
        Hence, it is possible to generate a straffing motion.
        The generated trajectory is then used as argument in the method :meth:`followTrajectory`.
        
        """
        start = self.get_center_of_feet_in_XY()

        end   = np.asarray(pos_in_XY)
        if relative_pos is True:
            end += start

        direction_vector = (end - start)
        path_length = np.linalg.norm(direction_vector)
        if angle is None:
            angle = np.arctan2(direction_vector[1], direction_vector[0])

        N = int(path_length/search_path_tolerance)
        traj = np.array([np.linspace(start[0], end[0], N), np.linspace(start[1], end[1], N), angle*np.ones(N)]).T

        self.followTrajectory(traj, verbose)


    def followTrajectory(self, trajectory, verbose=False):
        """ Generate a walking activity where the robot has to follow a desired trajectory.
        
        :param trajectory: The trajectory to follow
        :type  trajectory: (N,3)-array
        :param bool verbose: If one wants to get the log information sent by the :class:`FootTrajController`
        
        The trajectory is a (N,3)-array where:
        
        * The first column is the X trajectory in the planeXY,
        * The second column is the Y trajectory in the planeXY,
        * The third column is the angle trajectory around the normal of the planeXY.
        
        Not all points are used, only those which correspond to possible footsteps, based on the footsteps parameters (see :meth:`set_step_parameters`).
        These points are selected to build the ZMP points, footsteps positions and angles, and the waist angles.
        
        """
        l_start, r_start = self.get_lfoot_pose_in_XY(), self.get_rfoot_pose_in_XY()

        points  = traj2zmppoints(trajectory, self.length, self.side, l_start, r_start, self.start_foot)
        zmp_ref = zmppoints2zmptraj(points, self.step_time, self.dt)
        ftraj   = zmppoints2foottraj(points, self.step_time, self.ratio, self.height, self.dt, self.H_0_planeXY)
        wtraj   = zmppoints2waisttraj(points, self.step_time, self.dt, self.H_0_planeXY)

        if self.com_ctrl is not None:
            self.ctrl.task_updater.remove( self.com_ctrl )
        if self.feet_ctrl is not None:
            self.ctrl.task_updater.remove( self.feet_ctrl )

        self.com_ctrl = task_controller.ZMPController( self.com_task, self.dm, zmp_ref, self.RonQ, self.horizon, self.dt, self.H_0_planeXY, self.stride, self.gravity, self.height_ref, self.updatePxPu, self.use_swig_zmpy)
        self.ctrl.task_updater.register( self.com_ctrl )

        self.feet_ctrl = FootTrajController(self.lfoot_ctrl, self.rfoot_ctrl, self.lfoot_contacts, self.rfoot_contacts, ftraj, self.step_time, self.ratio, self.dt, self.start_foot, self.contact_as_objective, verbose)
        self.ctrl.task_updater.register( self.feet_ctrl )

        self.waist_rot_ctrl.set_new_trajectory( wtraj )

        return zmp_ref


    def moveOneFoot(self, start_foot, length, side_length, angle=None):
        """ Genrate a motion where the robot moves only one foot.
        
        :param string start_foot: The starting foot when the walk activity starts ('left' or 'right')
        :param double length: The distance to move the feet in the forward direction (based on current orientation)
        :param double side_length: The distance to move the feet in the lateral direction (based on current orientation)
        :param double angle: The final orientation of the foot in the planeXY; if None, the angle is in the forward direction
        
        """
        l_start      = self.get_lfoot_pose_in_XY()
        r_start      = self.get_rfoot_pose_in_XY()
        central_pose = (l_start + r_start )/2.
        angle_start  = central_pose[2]
        
        forward_direction = np.array( [ np.cos(angle_start), np.sin(angle_start) ] )
        left_direction    = np.array( [-np.sin(angle_start), np.cos(angle_start) ] )
        
        if angle is None:
            angle = angle_start

        if start_foot == 'left':
            lpose = length*forward_direction + side_length*left_direction
            points = [l_start, r_start, (lpose[0], lpose[1], angle)]
        elif start_foot == 'right':
            rpose = length*forward_direction - side_length*left_direction # minus because it is right foot
            points = [r_start, l_start, (rpose[0], rpose[1], angle)]
        else:
            raise ValueError

        zmp_ref = zmppoints2zmptraj(points, self.step_time, self.dt)
        ftraj   = zmppoints2foottraj(points, self.step_time, self.ratio, self.height, self.dt, self.H_0_planeXY)
        wtraj   = zmppoints2waisttraj(points, self.step_time, self.dt, self.H_0_planeXY)

        if self.com_ctrl is not None:
            self.ctrl.task_updater.remove( self.com_ctrl )
        if self.feet_ctrl is not None:
            self.ctrl.task_updater.remove( self.feet_ctrl )

        self.com_ctrl = task_controller.ZMPController( self.com_task, self.dm, zmp_ref, self.RonQ, self.horizon, self.dt, self.H_0_planeXY, self.stride, self.gravity, self.height_ref, self.updatePxPu, self.use_swig_zmpy)
        self.ctrl.task_updater.register( self.com_ctrl )

        self.feet_ctrl = FootTrajController(self.lfoot_ctrl, self.rfoot_ctrl, self.lfoot_contacts, self.rfoot_contacts, ftraj, self.step_time, self.ratio, self.dt, self.start_foot, self.contact_as_objective)
        self.ctrl.task_updater.register( self.feet_ctrl )

        self.waist_rot_ctrl.set_new_trajectory( wtraj )

        return zmp_ref


    def wait_for_end_of_walking(self, period=1e-3):
        """ Blocking method, until the walking activity is done.
        
        :param double period: Time interval between two test of :meth:`is_walking`
        
        """
        while 1:
            if not self.is_walking():
                break
            time.sleep(period)

    def wait_for_double_support(self, period=1e-3):
        """ Blocking method, until the robot is in double support configuration.
        
        :param double period: Time interval between two test of :meth:`is_on_double_support`
        
        """
        while 1:
            if self.is_on_double_support():
                break
            time.sleep(period)



