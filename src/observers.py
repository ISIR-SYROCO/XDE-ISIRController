#!/xde

import deploy.deployer as ddeployer
import xdefw.rtt
import rtt_interface

import numpy as np
import lgsm

import time
import os


def getAdjacencyMatrix(tw):
    return lgsm.np.matrix(
    [[ 0    ,-tw[2], tw[1], 0    , 0    , 0    ],
     [ tw[2], 0    ,-tw[0], 0    , 0    , 0    ],
     [-tw[1], tw[0], 0    , 0    , 0    , 0    ],
     [ 0    ,-tw[5], tw[4], 0    ,-tw[2], tw[1]],
     [ tw[5], 0    ,-tw[3], tw[2], 0    ,-tw[0]],
     [-tw[4], tw[3], 0    ,-tw[1], tw[0], 0    ]]
    )


def getDerivativeAdjoint(adj, tw):
    return adj * getAdjacencyMatrix(tw)



################################################################################
################################################################################
# Useful function that can be use alone or inside some recorder
################################################################################
################################################################################
def computeZMPLIPM(CoMPosition, CoMVelocity, CoMAcceleration, H_plane_0, gravity=9.81):
    pcom = H_plane_0               * CoMPosition
    vcom = H_plane_0.getRotation() * CoMVelocity
    acom = H_plane_0.getRotation() * CoMAcceleration

    zmp  = pcom[0:2] - acom[0:2] * (pcom.item(2)/gravity)

    return np.array(zmp).flatten()




def get_alldq(dynModel):
    dq = lgsm.zeros(dynModel.nbDofs())
    if dynModel.hasFixedRoot():
        dq[:]   = dynModel.getJointVelocities().copy()
    else:
        dq[0:6] = dynModel.getFreeFlyerVelocity().copy()
        dq[6: ] = dynModel.getJointVelocities().copy()
    return dq


def _getSegmentCoMPropertiesIn0(dm, index):
        H_0_s        = dm.getSegmentPosition(index)
        p_s_scom     = dm.getSegmentCoM(index)
        J_s_0_s      = dm.getSegmentJacobian(index)
        T_s_0_s      = dm.getSegmentVelocity(index)
        dJ_s_0_s__dq = dm.getSegmentJdotQdot(index)

        #inertia info
        m         = dm.getSegmentMass(index)
        Inertia   = lgsm.np.diag( [v for v in dm.getSegmentMomentsOfInertia(index).flat] )

        # CoM Position
        p_0_scom     = H_0_s * p_s_scom

        # CoM Jacobian
        H_s_scomNoRot = lgsm.Displacement()
        H_s_scomNoRot.setTranslation(p_s_scom)
        H_s_scomNoRot.setRotation(H_0_s.getRotation().inverse())
        Ad_scomNoRot_s = H_s_scomNoRot.inverse().adjoint()
        J_scomNotRot_0_0 = Ad_scomNoRot_s * J_s_0_s     #scomNoRot has the same orientation as 0, so J_scomNotRot_0_scomNotRot = J_scomNotRot_0_0
        J_scom_0_0 = J_scomNotRot_0_0 #[3:6,:]
        
        # CoM dJacobian_dq
        T_scomNoRot_0_s  = lgsm.Twist(T_s_0_s)
        T_scomNoRot_0_s.setLinearVelocity(lgsm.zeros(3))
        dAd_scomNoRot_0_s = getDerivativeAdjoint(Ad_scomNoRot_s, T_scomNoRot_0_s)
        dJ_scomNotRot_0_0__dq = Ad_scomNoRot_s * dJ_s_0_s__dq   +   dAd_scomNoRot_0_s * T_s_0_s
        dJ_scom_0_0__dq = dJ_scomNotRot_0_0__dq #[3:6,:]

        return m, Inertia, p_0_scom, J_scom_0_0, dJ_scom_0_0__dq


def computeZMP(dm, ddq, H_plane_0, gravity_vector):
    dq          = get_alldq(dm)

    Resultante0 = lgsm.zeros(3)
    Moment0     = lgsm.zeros(3)
    for i in range(dm.nbSegments()):
        m, Inertia, p_0_scom, J_scom_0_0, dJ_scom_0_0__dq = _getSegmentCoMPropertiesIn0(dm, i)

        V_scom_0_0  = J_scom_0_0 * dq
        dV_scom_0_0 = J_scom_0_0 * ddq  +  dJ_scom_0_0__dq

        linacc = dV_scom_0_0[3:6] - gravity_vector
        rotacc = dV_scom_0_0[0:3]
        rotvel = V_scom_0_0[0:3]

        R0 = m * linacc
        M0 = m * lgsm.crossprod(p_0_scom, linacc)  +  (Inertia * rotacc  - lgsm.crossprod((Inertia * rotvel), rotvel) )

        Resultante0 += R0
        Moment0     += M0

    n = gravity_vector/lgsm.norm(gravity_vector)
    zmp = lgsm.crossprod(n, Moment0) / lgsm.dotprod(n, Resultante0)
    zmp_XY = H_plane_0 * zmp

    return np.array(zmp_XY[0:2]).flatten()



def getOrthogonalityData(first, second, tol=1e-6):
    sinThetaOrthoVec = lgsm.crossprod(first, second)
    cosTheta         = lgsm.dotprod(  first, second)
    sinTheta         = lgsm.norm(sinThetaOrthoVec)
    theta            = lgsm.np.arctan2(sinTheta, cosTheta)

    if sinTheta > tol:
        OrthoVec = sinThetaOrthoVec / sinTheta
    else:
        OrthoVec = None

    return OrthoVec, theta, sinTheta, cosTheta


def rotationToAlignFirstToSecond(first, second):
    ov, t, st, ct = getOrthogonalityData(first, second)

    if ov is not None:
        return lgsm.Quaternion.fromAxisAngle( ov, t )
    else:
        if ct >= 0:
            return lgsm.Quaternion(1,0,0,0)
        else:
            return lgsm.Quaternion(0,1,0,0)


def lookAt(_from, _to, _up):
    # vec aligned with -z
    # up  aligned with y

    vec  = _to - _from
    nvec = lgsm.normalize(vec)
    minusZ = lgsm.vector(0,0,-1)

    Q_0_Z  = rotationToAlignFirstToSecond(minusZ, vec)

    Up_in_Z = Q_0_Z.inverse() * _up

    ov_Y = lgsm.vector(1,0,0) # = getOrthogonalityData(minusZ, Y)
    ov_UP, t, st, ct = getOrthogonalityData(minusZ, Up_in_Z)
    if ov_UP is not None:
        ov_diff, t_diff, st_diff, ct_diff = getOrthogonalityData(ov_Y, ov_UP)
        Q_Z_UP = lgsm.Quaternion.fromAxisAngle( ov_diff, t_diff)
    else:
        Q_Z_UP = lgsm.Quaternion()

    H = lgsm.Displacement()
    H.setTranslation(_from)
    H.setRotation(Q_0_Z * Q_Z_UP)
    return H


################################################################################
################################################################################
# RECORDERS
################################################################################
################################################################################
class Recorder(object):
    def __init__(self):
        self._record = []

    def update(self, tick):
        raise NotImplementedError

    def save_record(self, rec):
        self._record.append(rec)

    def get_record(self):
        return self._record


class StateObserver(Recorder):
    def __init__(self, dynModel):
        Recorder.__init__(self)
        self.dynModel = dynModel

    def update(self, tick):
        if self.dm.hasFixedRoot():
            state = (self.dm.getJointPositions(), self.dm.getJointVelocities())
        else:
            state = (self.dm.getFreeFlyerPosition(), self.dm.getJointPositions(), self.dm.getFreeFlyerVelocity(), self.dm.getJointVelocities())
        self.save_record(state)


class JointPositionsObserver(Recorder):
    def __init__(self, dynModel):
        Recorder.__init__(self)
        
        self.dynModel = dynModel

    def update(self, tick):
        pos = np.array(self.dynModel.getJointPositions()).flatten()
        self.save_record(pos)


class FramePoseObserver(Recorder):
    def __init__(self, dynModel, seg_name, H_seg_frame):
        Recorder.__init__(self)
        self.dynModel = dynModel
        self.seg_idx  = self.dynModel.getSegmentIndex(seg_name)
        self.H_s_f    = H_seg_frame

    def update(self, tick):
        self.save_record(self.dynModel.getSegmentPosition(self.seg_idx) * self.H_s_f)


class CoMPositionObserver(Recorder):
    def __init__(self, dynModel):
        Recorder.__init__(self)
        self.dynModel   = dynModel

    def update(self, trick):
        self.save_record(np.array(self.dynModel.getCoMPosition()).flatten())



class ZMPLIPMPositionObserver(Recorder):
    def __init__(self, dynModel, H_0_plane, dt, gravity):
        Recorder.__init__(self)

        self.dynModel  = dynModel
        self.dt        = dt
        self.gravity   = gravity
        self.H_plane_0         = H_0_plane.inverse()
        self.prev_CoMVelocity  = self.dynModel.getCoMVelocity()


    def update(self, tick):
        CoMPosition           = self.dynModel.getCoMPosition()
        CoMVelocity           = self.dynModel.getCoMVelocity()
        CoMAcceleration       = (CoMVelocity - self.prev_CoMVelocity)/self.dt
        self.prev_CoMVelocity = CoMVelocity.copy()
        zmplipm = computeZMPLIPM(CoMPosition, CoMVelocity, CoMAcceleration, self.H_plane_0, self.gravity)
        self.save_record(zmplipm)


class ZMPPositionObserver(Recorder):
    def __init__(self, dynModel, H_0_plane, dt, gravity, up=None):
        Recorder.__init__(self)

        self.dynModel  = dynModel
        self.dt        = dt
        self.gravity   = gravity
        if up is None:
            up = lgsm.vector([0,0,1])
        self.gravity_vector = - self.gravity * up

        self.H_plane_0      = H_0_plane.inverse()

        self.prev_dq = get_alldq(self.dynModel)

    def update(self, tick):
        dq  = get_alldq(self.dynModel)
        ddq = (dq - self.prev_dq)/self.dt
        self.prev_dq = dq
        zmp = computeZMP(self.dynModel, ddq, self.H_plane_0, self.gravity_vector)
        self.save_record(zmp)







class ScreenShotObserver(Recorder):
    def __init__(self, world_manager, rec_folder,  x=800, y=600, cam_traj=None):
        Recorder.__init__(self)
        self.wm = world_manager
        self.rec_folder = rec_folder
        try:
            os.mkdir(rec_folder)
        except:
            pass
        
        self.idx = 0
        
        self.cam_traj = cam_traj
        if self.cam_traj is None:
            self.cam_traj = []
        
        self.cam_name    = "mainViewportBaseCamera"
        self.window_name = "mainWindow"
        
        self.wm.resizeWindow(self.window_name, x, y)

    def update(self, tick):
        
        if self.idx < len(self.cam_traj):
            self.wm.graph_scn.CameraInterface.setDisplacementRelative(self.cam_name, self.cam_traj[self.idx] )
        
        img_name = self.rec_folder + os.sep + "{:06d}.png".format(self.idx)
        self.wm.graph.s.Viewer.takeScreenShot(self.window_name, img_name)
        self.idx += 1





class TorqueObserver(xdefw.rtt.Task, Recorder):
    def __init__(self, ctrl, name="TorqueObserver_OrocosTask"):
        super(TorqueObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        Recorder.__init__(self)

        self.tau_in = self.addCreateInputPort("tau", "VectorXd")
        ctrl.getPort("tau").connectTo(self.tau_in)


    def update(self, tick):
        tau, tau_ok = self.tau_in.read()
        if tau_ok:
            self.save_record(np.array(tau).flatten())




















#class ContactDistanceObserver(xdefw.rtt.Task):
#    def __init__(self, cinfo, physic_agent, sync):
#        name = "ContactDistanceObserver"
#        super(ContactDistanceObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
#        
#        self.cinfo = cinfo
#        
#        self.cport = self.addCreateInputPort("in_contact_info", "SMsg", True)
#        
#        self.cinfo.port.connectTo(self.cport)
#        
#        self.tick_out = self.addCreateOutputPort("tick.out", "int")
#        physic_agent.addCreateInputPort(name+".tick", "int")
#        if sync is not None:      # if not None, tell physic to wait for observer before continuing
#            sync.addEvent(name+".tick")
#        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

#    def startHook(self):
#        self.gaps = []

#    def stopHook(self):
#        pass

#    def updateHook(self):
#        smsg, smsg_ok = self.cport.read()
#        if smsg_ok:
#            gap = []
#            for c in smsg.cpt:
#                gap.append(c.gap)
#            self.gaps.append(gap)
#            self.tick_out.write(0)


#    def plot(self):
#        import pylab as pl
#        
#        max_gaps = max([len(g) for g in self.gaps])
#        dists = np.zeros((len(self.gaps), max_gaps))
#        
#        for i in range(len(dists)):
#            dists[i, :len(self.gaps[i])] = self.gaps[i]

#        pl.figure()
#        pl.plot(dists)
#        
#        pl.show()

































