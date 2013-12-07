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


##### OBSTACLES
sphereWorld = xrl.createWorldFromUrdfFile("resources/sphere.xml", "sphere1", [1.,.8,.5,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(sphereWorld)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
robot.enableContactWithRobot("sphere1", True)
#wm.contact.showContacts([(rname+"."+b,"sphere1.sphere") for b in ["link_x", "link_y", "link_z"]]) # to display contact


##### CTRL
import xde_isir_controller  as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", True)
ctrl.controller.takeIntoAccountGravity(False)
#jointConst  = ctrl.add_constraint( xic.JointLimitConstraint(ctrl.getModel(), .2) )

contConst   = ctrl.add_constraint( xic.ContactAvoidanceConstraint(ctrl.getModel(), .2, 0.2) )
contConstUpdater = ctrl.add_updater(xic.ContactAvoidanceConstraintUpdater(contConst, ctrl.getModel(), wm.phy))
contConstUpdater.add_contactAvoidance("sphere1.sphere", "robot.link_x")
contConstUpdater.add_contactAvoidance("sphere1.sphere", "robot.link_y")
contConstUpdater.add_contactAvoidance("sphere1.sphere", "robot.link_z")

#gposdes = lgsm.zeros(N)
#gposdes[1] = 1.2
#gveldes = lgsm.zeros(N)
fTask = ctrl.createFrameTask("frametask", "robot.link_z", lgsm.Displacement(), w=1., kp=5.)
fTask.setPosition(lgsm.Displacement(1.,.8,.5,1,0,0,0))


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


