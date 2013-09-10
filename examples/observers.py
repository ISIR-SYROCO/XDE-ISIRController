#!/xde

import deploy.deployer as ddeployer
import xdefw.rtt
import rtt_interface

import numpy as np
import lgsm

import time



class ContactDistanceObserver(xdefw.rtt.Task):
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







class JointPositionsObserver(xdefw.rtt.Task):
    def __init__(self, robot, physic_agent, sync):
        name = "JointPositionsObserver"
        super(JointPositionsObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        
        self.robot = robot
        
        self.tick_in = self.addCreateInputPort("tick.in", "int", True)
        physic_agent.getPort("tick").connectTo(self.tick_in)
        
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync is not None:      # if not None, tell physic to wait for observer before continuing
            sync.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self.positions = []

    def stopHook(self):
        pass

    def updateHook(self):
        v, v_ok = self.tick_in.read()
        if v_ok:
            pos = np.array(self.robot.getJointPositions()).flatten()
            self.positions.append(pos)
            self.tick_out.write(0)


    def plot(self):
        import pylab as pl
        
        pos = np.array(self.positions)

        pl.figure()
        pl.plot(pos)
        pl.legend([str(i) for i in range(pos.shape[1])])
        
        pl.show()






class TorqueObserver(xdefw.rtt.Task):
    def __init__(self, ctrl, physic_agent, sync):
        name = "TorqueObserver"
        super(TorqueObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        
        self.tau_in = self.addCreateInputPort("tau", "VectorXd", True)
        ctrl.getPort("tau").connectTo(self.tau_in)
        
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync is not None:      # if not None, tell physic to wait for observer before continuing
            sync.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self.tau = []

    def stopHook(self):
        pass

    def updateHook(self):
        tau, tau_ok = self.tau_in.read()
        if tau_ok:
            self.tau.append(np.array(tau).flatten())
            self.tick_out.write(0)


    def plot(self):
        import pylab as pl
        
        tau = np.array(self.tau)

        pl.figure()
        pl.plot(tau)
        pl.legend([str(i) for i in range(tau.shape[1])])
        
        pl.show()




class FramePoseObserver(xdefw.rtt.Task):
    def __init__(self, robot, seg_name, H_seg_frame, physic_agent, sync):
        name = "FramePoseObserver"
        super(FramePoseObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        
        self.robot   = robot
        self.seg_name = seg_name
        self.H_s_f   = H_seg_frame
        self.tick_in = self.addCreateInputPort("tick.in", "int", True)      # the best would be to get info from physic connectors
        physic_agent.getPort("tick").connectTo(self.tick_in)
        
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync is not None:      # if not None, tell physic to wait for observer before continuing
            sync.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self.pose = []

    def stopHook(self):
        pass

    def updateHook(self):
        tick, tick_ok = self.tick_in.read()
        if tick_ok:
            self.pose.append(self.robot.getSegmentPosition2(self.seg_name) * self.H_s_f)
            self.tick_out.write(0)


    def plot(self):
        import pylab as pl
        
#        print self.pose
        pose = np.array(self.pose)

        pl.figure()
        pl.plot(pose)
        pl.legend(['tx', 'ty', 'tz', 'w', 'rx', 'ry', 'rz'])
        
        pl.show()




class CoMPositionObserver(xdefw.rtt.Task):
    def __init__(self, dynModel, physic_agent, sync):
        name = "CoMPositionObserver"
        super(CoMPositionObserver, self).__init__(rtt_interface.PyTaskFactory.CreateTask(name))
        
        self.dynModel   = dynModel
        self.tick_in = self.addCreateInputPort("tick.in", "int", True) # the best would be to get info from physic connectors
        physic_agent.getPort("tick").connectTo(self.tick_in)
        
        self.tick_out = self.addCreateOutputPort("tick.out", "int")
        physic_agent.addCreateInputPort(name+".tick", "int")
        if sync is not None:      # if not None, tell physic to wait for observer before continuing
            sync.addEvent(name+".tick")
        self.tick_out.connectTo(physic_agent.getPort(name+".tick")) # connection from observer to physic, to tick when job is done

    def startHook(self):
        self.pos = []

    def stopHook(self):
        pass

    def updateHook(self):
        tick, tick_ok = self.tick_in.read()
        if tick_ok:
            self.pos.append(np.array(self.dynModel.getCoMPosition()).flatten())
            self.tick_out.write(0)


    def plot(self):
        import pylab as pl
        
        pos = np.array(self.pos)

        pl.figure()
        pl.plot(pos)
        pl.legend(['x', 'y', 'z'])
        
        pl.show()



