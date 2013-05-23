#!/xde

import deploy.deployer as ddeployer
import dsimi.rtt
import rtt_interface

import numpy as np
import lgsm

import time




class Recorder(dsimi.rtt.Task):
    def __init__(self, name, physic_agent, sync_connector=None):
        super(Recorder, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        self.tick_in  = self.addCreateInputPort("tick.in", "int", True)
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        
        physic_agent.getPort("tick").connectTo(self.tick_in)    # connection from physic to observer
        
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync_connector is not None:      # if not None, tell physic to wait for observer before continuing
            sync_connector.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self._record   = []

    def stopHook(self):
        pass

    def updateHook(self):
        self.tick, tick_ok = self.tick_in.read()    # <--- get info
        if tick_ok:
            self.doUpdateHook()
            self.tick_out.write(self.tick)          # ---> tell that job done

    def doUpdateHook(self):
        raise NotImplementedError

    def save_record(self, rec):
            self._record.append(rec)

    def get_record(self):
        return self._record







class ContactDistanceObserver(dsimi.rtt.Task):
    def __init__(self, cinfo, physic_agent, sync):
        name = "ContactDistanceObserver"
        super(ContactDistanceObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        
        self.cinfo = cinfo
        
        self.cport = self.addCreateInputPort("in_contact_info", "SMsg", True)
        
        self.cinfo.port.connectTo(self.cport)
        
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync is not None:      # if not None, tell physic to wait for observer before continuing
            sync.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self.gaps = []

    def stopHook(self):
        pass

    def updateHook(self):
        smsg, smsg_ok = self.cport.read()
        if smsg_ok:
            gap = []
            for c in smsg.cpt:
                gap.append(c.gap)
            self.gaps.append(gap)
            self.tick_out.write(0)


    def plot(self):
        import pylab as pl
        
        max_gaps = max([len(g) for g in self.gaps])
        dists = np.zeros((len(self.gaps), max_gaps))
        
        for i in range(len(dists)):
            dists[i, :len(self.gaps[i])] = self.gaps[i]

        pl.figure()
        pl.plot(dists)
        
        pl.show()







class JointPositionsObserver(Recorder):
    def __init__(self, robot, physic_agent, sync):
        Recorder.__init__(self, "JointPositionsObserver", physic_agent, sync)
        
        self.robot = robot

    def doUpdateHook(self):
        pos = np.array(self.robot.getJointPositions()).flatten()
        self.save_record(pos)


    def plot(self):
        import pylab as pl
        
        pos = np.array(self.get_record())

        pl.figure()
        pl.plot(pos)
        pl.legend([str(i) for i in range(pos.shape[1])])
        
        pl.show()






class TorqueObserver(Recorder):
    def __init__(self, ctrl, physic_agent, sync):
        Recorder.__init__(self, "TorqueObserver", physic_agent, sync)
        
        self.tau_in = self.addCreateInputPort("tau", "VectorXd")
        ctrl.getPort("tau").connectTo(self.tau_in)


    def updateHook(self):
        tau_ok = False
        while not tau_ok:
            tau, tau_ok = self.tau_in.read()
            if not tau_ok:
                time.sleep(0.001)
            else:
                self.save_record(np.array(tau).flatten())


    def plot(self):
        import pylab as pl
        
        tau = np.array(self.get_record())

        pl.figure()
        pl.plot(tau)
        pl.legend([str(i) for i in range(tau.shape[1])])
        
        pl.show()




class FramePoseObserver(Recorder):
    def __init__(self, robot, seg_name, H_seg_frame, physic_agent, sync):
        Recorder.__init__(self, "FramePoseObserver", physic_agent, sync)

        self.robot   = robot
        self.seg_name = seg_name
        self.H_s_f   = H_seg_frame

    def doUpdateHook(self):
        self.save_record(self.robot.getSegmentPosition2(self.seg_name) * self.H_s_f)


    def plot(self, reference_traj=None):
        import pylab as pl

        pose = np.array(self.get_record())

        pl.figure()
        pl.plot(pose)
        pl.legend(['tx', 'ty', 'tz', 'w', 'rx', 'ry', 'rz'])
        
        if reference_traj is not None:
            pl.plot(reference_traj, ls=":", lw=2)

        pl.show()




class CoMPositionObserver(Recorder):
    def __init__(self, dynModel, physic_agent, sync):
        Recorder.__init__(self, "CoMPositionObserver", physic_agent, sync)

        self.dynModel   = dynModel


    def doUpdateHook(self):
        self.save_record(np.array(self.dynModel.getCoMPosition()).flatten())



    def plot(self):
        import pylab as pl
        
        pos = np.array(self.get_record())

        pl.figure()
        pl.plot(pos)
        pl.legend(['x', 'y', 'z'])
        
        pl.show()



class ZMPLIPMPositionObserver(Recorder):
    def __init__(self, dynModel, H_0_plane, dt, gravity, physic_agent, sync):
        Recorder.__init__(self, "ZMPLIPMPositionObserver", physic_agent, sync)

        self.dynModel  = dynModel
        self.dt        = dt
        self.gravity   = gravity
        
        self.H_plane_0         = H_0_plane.inverse()
        self.H_plane_0_for_vel = lgsm.Displacement(lgsm.vector(0,0,0), self.H_plane_0.getRotation())
        
        self.prev_vcom      = self.dynModel.getCoMVelocity()
        self.prev_prev_vcom = self.dynModel.getCoMVelocity()


    def doUpdateHook(self):
        pcom = self.H_plane_0         * self.dynModel.getCoMPosition()
        vcom = self.H_plane_0_for_vel * self.dynModel.getCoMVelocity()
        acom = (vcom - self.prev_vcom)/self.dt

        self.prev_vcom = vcom

        zmp  = pcom[0:2] - acom[0:2] * (pcom.item(2)/self.gravity)

        self.save_record(np.array(zmp).flatten())



    def plot(self, zmp_ref=None):
        import pylab as pl
        
        pos = np.array(self.get_record())

        pl.figure()
        pl.plot(pos)
        pl.legend(['x', 'y'])
        
        if zmp_ref is not None:
            pl.plot(zmp_ref, ls=":", lw=2)
        
        pl.show()


import os

class ScreenShotObserver(Recorder):
    def __init__(self, world_manager, rec_folder,  x=800, y=600, cam_traj=None):

        self.wm = world_manager

        Recorder.__init__(self, "ScreenShotObserver", self.wm.phy, self.wm.icsync )

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
        
        d_g_c = self.wm.graph_scn.CameraInterface.getCameraDisplacement(self.cam_name)
        d_p_c = self.wm.graph_scn.CameraInterface.getCameraDisplacementInPhysicSpace(self.cam_name)
        
        self.d_g_p = d_g_c * d_p_c.inverse()
        
        self.wm.resizeWindow(self.window_name, x, y)


    def doUpdateHook(self):
        
        if self.idx < len(self.cam_traj):
            self.wm.graph_scn.CameraInterface.setCameraDisplacement(self.cam_name, self.d_g_p * self.cam_traj[self.idx] ) #,0.707,0.707,0,0
        
        img_name = self.rec_folder + os.sep + "{:06d}.png".format(self.idx)
        self.wm.graph.s.Viewer.takeScreenShot(self.window_name, img_name)
        self.idx += 1




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
        return lgsm.Quaternion( ov, t )
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
        Q_Z_UP = lgsm.Quaternion( ov_diff, t_diff)
    else:
        Q_Z_UP = lgsm.Quaternion()

    return lgsm.Displacement(_from, Q_0_Z * Q_Z_UP )


