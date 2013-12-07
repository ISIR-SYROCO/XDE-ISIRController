#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.3)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.001, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


##### OBSTACLES
sphereWorld = xrl.createWorldFromUrdfFile("resources/sphere.xml", "sphere1", [0.45,0,0.3,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(sphereWorld)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
robot.enableContactWithRobot("sphere1", True)
#wm.contact.showContacts([(rname+"."+b,"sphere1.sphere") for b in ["05", "06", "07"]]) # to display contact


##### CTRL
import xde_isir_controller  as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", False)

jointConst  = ctrl.add_constraint( xic.JointLimitConstraint(ctrl.getModel(), .2) )

contConst   = ctrl.add_constraint( xic.ContactAvoidanceConstraint(ctrl.getModel(), .2, 0.1) )
contConstUpdater = ctrl.add_updater(xic.ContactAvoidanceConstraintUpdater(contConst, ctrl.getModel(), wm.phy))
contConstUpdater.add_contactAvoidance("robot.04", "sphere1.sphere")
contConstUpdater.add_contactAvoidance("robot.05", "sphere1.sphere")
contConstUpdater.add_contactAvoidance("robot.06", "sphere1.sphere")
contConstUpdater.add_contactAvoidance("robot.07", "sphere1.sphere")


gposdes = lgsm.zeros(N)
gposdes[1] = 1.2
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", "INTERNAL", w=1., kp=20.)
fullTask.set_q(gposdes)
fullTask.set_qdot(gveldes)


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


