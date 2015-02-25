#!/xde

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time
import xdefw.interactive

pi = lgsm.np.pi

##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  800, 600, 50, 50)

mu_sys = 0.5

##### GROUND
ground_fixed_base = True
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], ground_fixed_base, 0.001, 0.001)
wm.addWorld(groundWorld)

##### ROBOT
rname = "robot"
robot_fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.romeo_collision, rname, [0,0,0.88,1,0,0,0], robot_fixed_base, 0.001, 0.001)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
#robot.enableGravity(False)
N  = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)
jmap     = xrl.getJointMapping(xr.romeo_collision, robot)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, mu_sys)
robot.enableContactWithBody("ground.ground", True)
#wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_ankle", "r_ankle"]]) # to display contact


##### SET INITIAL STATE
qinit = lgsm.zeros(N)
for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.)]:
#for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.), ("LHipPitch", -pi/15.), ("RHipPitch", -pi/15.), ("LKneePitch", 2*pi/15.), ("RKneePitch", 2*pi/15.), ("RAnklePitch", -pi/15.), ("LAnklePitch", -pi/15.)]:
    qinit[jmap[rname+"."+name]] = val

#for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.)]:
#    qinit[jmap[rname+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))

##### CONTROLLER
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog", False)

fullTask = ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=9.0, q_des=qinit)
waistTask   = ctrl.createFrameTask("body", rname+'.body', lgsm.Displacement(), "RXYZ", w=1., kp=36., pose_des=lgsm.Displacement(0,0,0.85,1,0,0,0))

foot_contact_pt0 = lgsm.Displacement([0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt1 = lgsm.Displacement([-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt2 = lgsm.Displacement([0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt3 = lgsm.Displacement([-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
toe_contact_pt0 = lgsm.Displacement([0.05, 0.03, -0.01, 0.0, 1.0, 0.0, 0.0])
toe_contact_pt1 = lgsm.Displacement([0.05, -0.03, -0.01, 0.0, 1.0, 0.0, 0.0])

ct_clf0 = ctrl.createContactTask("CLF0", rname+".l_ankle", foot_contact_pt0, mu_sys)
ct_clf1 = ctrl.createContactTask("CLF1", rname+".l_ankle", foot_contact_pt1, mu_sys)
ct_clf2 = ctrl.createContactTask("CLF2", rname+".l_ankle", foot_contact_pt2, mu_sys)
ct_clf3 = ctrl.createContactTask("CLF3", rname+".l_ankle", foot_contact_pt3, mu_sys)
#ctrl.createContactTask("CLF4", rname+".l_toe", toe_contact_pt0, mu_sys)
#ctrl.createContactTask("CLF5", rname+".l_toe", toe_contact_pt1, mu_sys)

ct_crf0 = ctrl.createContactTask("CRF0", rname+".r_ankle", foot_contact_pt0, mu_sys)
ct_crf1 = ctrl.createContactTask("CRF1", rname+".r_ankle", foot_contact_pt1, mu_sys)
ct_crf2 = ctrl.createContactTask("CRF2", rname+".r_ankle", foot_contact_pt2, mu_sys)
ct_crf3 = ctrl.createContactTask("CRF3", rname+".r_ankle", foot_contact_pt3, mu_sys)
#ctrl.createContactTask("CRF4", rname+".r_toe", toe_contact_pt0, mu_sys)
#ctrl.createContactTask("CRF5", rname+".r_toe", toe_contact_pt1, mu_sys)

##### OBSERVERS
torque_obs = ctrl.add_updater(xic.observers.TorqueObserver(ctrl))
zmplipmpobs = ctrl.add_updater(xic.observers.ZMPLIPMPositionObserver(ctrl.getModel(), lgsm.Displacement(), dt, 9.81) )


##### ADD FRAMES
"""
wm.markers.addBodyMarker(rname+".l_ankle", "CLF0", lgsm.Displacement(foot_contact_pt0))
wm.markers.addBodyMarker(rname+".l_ankle", "CLF1", lgsm.Displacement(foot_contact_pt1))
wm.markers.addBodyMarker(rname+".l_ankle", "CLF2", lgsm.Displacement(foot_contact_pt2))
wm.markers.addBodyMarker(rname+".l_ankle", "CLF3", lgsm.Displacement(foot_contact_pt3))
wm.markers.addBodyMarker(rname+".r_ankle", "CLR0", lgsm.Displacement(foot_contact_pt0))
wm.markers.addBodyMarker(rname+".r_ankle", "CLR1", lgsm.Displacement(foot_contact_pt1))
wm.markers.addBodyMarker(rname+".r_ankle", "CLR2", lgsm.Displacement(foot_contact_pt2))
wm.markers.addBodyMarker(rname+".r_ankle", "CLR3", lgsm.Displacement(foot_contact_pt3))
"""

##### SIMULATE
ctrl.s.start()
wm.startAgents()
wm.phy.s.agent.triggerUpdate()

ctrl.waitSimTime(3.0, dt)

wm.stopAgents()
ctrl.s.stop()

#xdefw.interactive.shell_console()()

import pylab as pl
torque = torque_obs.get_record()
pl.plot(torque)
pl.show()

zmplipm = zmplipmpobs.get_record()
pl.plot(zmplipm)
pl.show()
