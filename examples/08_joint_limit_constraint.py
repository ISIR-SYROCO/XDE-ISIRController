#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


##### CTRL
import xde_isir_controller  as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog", True)

jointConst  = ctrl.add_constraint( xic.JointLimitConstraint(ctrl.getModel(), -0.5 * lgsm.ones(N), 0.8 * lgsm.ones(N), .1) )
#jointConst.setJointLimits(-0.5 * lgsm.ones(N), 0.8 * lgsm.ones(N))


gposdes = 1.2 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", "INTERNAL", w=1., kp=20.)
fullTask.set_q(gposdes)
fullTask.set_qdot(gveldes)


##### OBSERVERS
jpobs = ctrl.add_updater(xic.observers.JointPositionsObserver(ctrl.getModel()))

###### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()


time.sleep(3.)
jointConst.setHorizonOfPrediction(.5)
fullTask.set_q( -1.2 * lgsm.ones(N))
time.sleep(4.)
jointConst.setHorizonOfPrediction(1.)
jointConst.setJointUpperLimit(3, 0.2)
fullTask.set_q( 1.2 * lgsm.ones(N))
time.sleep(5.)

wm.stopAgents()
ctrl.s.stop()



##### RESULTS
import pylab as pl

jpos = jpobs.get_record()
pl.plot(jpos)
pl.show()


