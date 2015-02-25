#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.5)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile("resources/Robot3T.xml", rname, [0,0,0,1,0,0,0], True, 0.001, 0.001, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(False)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


robot.setJointPositions(lgsm.vector(.5,.5,.5))
dynModel.setJointPositions(lgsm.vector(.5,.5,.5))
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
#robot.enableContactWithRobot("sphere1", True)
#wm.contact.showContacts([(rname+"."+b,"sphere1.sphere") for b in ["link_x", "link_y", "link_z"]]) # to display contact

robot.enableAllContacts(True)
##### CTRL
import xde_isir_controller  as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", True)
ctrl.controller.takeIntoAccountGravity(False)
#jointConst  = ctrl.add_constraint( xic.JointLimitConstraint(ctrl.getModel(), .2) )

contConst   = ctrl.add_constraint( xic.ContactAvoidanceConstraint(ctrl.getModel(), .2, 0.1) )
contConstUpdater = ctrl.add_updater(xic.ContactAvoidanceConstraintUpdater(contConst, ctrl.getModel(), wm.phy))
#contConstUpdater.add_contactAvoidance("robot.link_x", "robot.link_y")
contConstUpdater.add_contactAvoidance("robot.link_x", "robot.link_z")
contConstUpdater.add_contactAvoidance("robot.link_y", "robot.link_z")

gposdes = lgsm.zeros(N)
gveldes = lgsm.zeros(N)
fTask = ctrl.createFullTask("frametask", "INTERNAL", w=1., kp=5., set_q=gposdes, set_qdot=gveldes)


##### OBSERVERS
jpobs = ctrl.add_updater(xic.observers.JointVelocitiesObserver(ctrl.getModel()))

###### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()


time.sleep(10.)


wm.stopAgents()
ctrl.s.stop()



##### RESULTS
import pylab as pl

jpos = jpobs.get_record()
pl.plot(jpos)
pl.show()


