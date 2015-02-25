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
robotWorld = xrl.createWorldFromUrdfFile(xr.romeo_collision, rname, [0,0,0.88,1,0,0,0], robot_fixed_base, 0.003, 0.001)
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
for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.), ("LElbowRoll", -pi/2.), ("RElbowRoll", pi/2.)]:
#for name, val in [("LShoulderPitch", pi/4.), ("RShoulderPitch", pi/4.), ("LShoulderYaw", pi/8.), ("RShoulderYaw", -pi/8.), ("LHipPitch", -pi/15.), ("RHipPitch", -pi/15.), ("LKneePitch", 2*pi/15.), ("RKneePitch", 2*pi/15.), ("RAnklePitch", -pi/15.), ("LAnklePitch", -pi/15.)]:
    qinit[jmap[rname+"."+name]] = val

#for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.)]:
#    qinit[jmap[rname+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))

##### CONTROLLER
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", False)

##### SET CONSTRAINTS
#torqueConst = ctrl.add_constraint(xic.TorqueLimitConstraint(ctrl.getModel(), 80.*lgsm.ones(N) ) )
#jointConst  = ctrl.add_constraint(xic.JointLimitConstraint(ctrl.getModel(), .2 ) )

##### SET TASKS
fullTask = ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=9.0, q_des=qinit)
back_dofs   = [jmap[rname+"."+n] for n in ['TrunkYaw']]
backTask    = ctrl.createPartialTask("back", back_dofs, w=0.01, kp=25., q_des=lgsm.zeros(3))

foot_contact_pt0 = lgsm.Displacement([0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt1 = lgsm.Displacement([-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt2 = lgsm.Displacement([0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
foot_contact_pt3 = lgsm.Displacement([-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0])
#toe_contact_pt0 = lgsm.Displacement([0.06, 0.03, -0.01, 0.0, 1.0, 0.0, 0.0])
#toe_contact_pt1 = lgsm.Displacement([0.06, -0.03, -0.01, 0.0, 1.0, 0.0, 0.0])

foot_contact_sole = lgsm.Displacement([0.04, 0.0, -0.06, 1.0, 0.0, 0.0, 0.0])

ct_clf0 = ctrl.createContactTask("CLF0", rname+".l_ankle", foot_contact_pt0, mu_sys)
ct_clf1 = ctrl.createContactTask("CLF1", rname+".l_ankle", foot_contact_pt1, mu_sys)
ct_clf2 = ctrl.createContactTask("CLF2", rname+".l_ankle", foot_contact_pt2, mu_sys)
ct_clf3 = ctrl.createContactTask("CLF3", rname+".l_ankle", foot_contact_pt3, mu_sys)

l_contacts = [ct_clf0, ct_clf1, ct_clf2, ct_clf3]
#ctrl.createContactTask("CLF4", rname+".l_toe", toe_contact_pt0, mu_sys)
#ctrl.createContactTask("CLF5", rname+".l_toe", toe_contact_pt1, mu_sys)

ct_crf0 = ctrl.createContactTask("CRF0", rname+".r_ankle", foot_contact_pt0, mu_sys)
ct_crf1 = ctrl.createContactTask("CRF1", rname+".r_ankle", foot_contact_pt1, mu_sys)
ct_crf2 = ctrl.createContactTask("CRF2", rname+".r_ankle", foot_contact_pt2, mu_sys)
ct_crf3 = ctrl.createContactTask("CRF3", rname+".r_ankle", foot_contact_pt3, mu_sys)
#ctrl.createContactTask("CRF4", rname+".r_toe", toe_contact_pt0, mu_sys)
#ctrl.createContactTask("CRF5", rname+".r_toe", toe_contact_pt1, mu_sys)

r_contacts = [ct_crf0, ct_crf1, ct_crf2, ct_crf3]

walkingActivity = xic.walk.WalkingActivity(ctrl, dt,
                                    rname+".l_ankle", foot_contact_sole, [ct_clf0, ct_clf1, ct_clf2, ct_clf3],
                                    rname+".r_ankle", foot_contact_sole, [ct_crf0, ct_crf1, ct_crf2, ct_crf3],
                                    rname+'.body', lgsm.Displacement(0,0,0,1,0,0,0), lgsm.Displacement(0,0,0.83,1,0,0,0),
                                    H_0_planeXY=lgsm.Displacement(0,0,0.002,1,0,0,0), weight=10., contact_as_objective=True)

walkingActivity.set_zmp_control_parameters(RonQ=1e-6, horizon=2.0, stride=2, gravity=9.81, height_ref=0.83, updatePxPu=1e-3)
walkingActivity.set_step_parameters(length=0.20, side=0.10, height=0.02, time=1.0, ratio=0.9, start_foot="left")
#walkingTask.stayIdle()


zmp_ref = walkingActivity.goTo([1.0,0.0], relative_pos=True)

##### OBSERVERS
torque_obs = ctrl.add_updater(xic.observers.TorqueObserver(ctrl))
zmplipmpobs = ctrl.add_updater(xic.observers.ZMPLIPMPositionObserver(ctrl.getModel(), lgsm.Displacement(), dt, 9.81) )
CoM_obs = ctrl.add_updater(xic.observers.CoMPositionObserver(ctrl.getModel()))

##### ADD FRAMES
"""
wm.markers.addBodyMarker(rname+".l_ankle", "CLF0", foot_contact_pt0)
wm.markers.addBodyMarker(rname+".l_ankle", "CLF1", foot_contact_pt1)
wm.markers.addBodyMarker(rname+".l_ankle", "CLF2", foot_contact_pt2)
wm.markers.addBodyMarker(rname+".l_ankle", "CLF3", foot_contact_pt3)
wm.markers.addBodyMarker(rname+".l_ankle", "l_sole_contact", foot_contact_sole)
wm.markers.addBodyMarker(rname+".r_ankle", "CLR0", foot_contact_pt0)
wm.markers.addBodyMarker(rname+".r_ankle", "CLR1", foot_contact_pt1)
wm.markers.addBodyMarker(rname+".r_ankle", "CLR2", foot_contact_pt2)
wm.markers.addBodyMarker(rname+".r_ankle", "CLR3", foot_contact_pt3)
"""

##### SIMULATE
ctrl.s.start()
wm.startAgents()
wm.phy.s.agent.triggerUpdate()

"""
cp = wm.phy.getPort("contacts")
ip = cp.antiClone()
ip.connectTo(cp)
#a,b = ip.read()
#xdefw.interactive.shell_console()()

for i in range(1000):
    a,b = ip.read() 
    if b is True:
      for c in a.cpt:
        print c.normalForce
    time.sleep(.01)

"""

ctrl.waitSimTime(10.0, dt)
wm.stopAgents()
ctrl.s.stop()

#xdefw.interactive.shell_console()()

import pylab as pl
pl.figure()
torque = torque_obs.get_record()
pl.plot(torque)

pl.figure()
com_pos = CoM_obs.get_record()
pl.plot(com_pos)

pl.figure()
zmplipm = zmplipmpobs.get_record()
pl.plot(zmplipm)
pl.plot(zmp_ref, ':')
pl.show()
