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
wm.resizeWindow("mainWindow", 640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


##### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog")


gposdes = 0.5 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", w=0.0001, kp=10.)  # create full task with a very low weight for reference posture
fullTask.set_q(gposdes)
fullTask.set_qdot(gveldes)


gposdes = lgsm.Displacement(.1,.1,.2,1,0,0,0)
gveldes = lgsm.Twist()
CoMTask = ctrl.createCoMTask("CoM", "XYZ", w=1., kp=20.) # dofs can be replaced by combination of
CoMTask.setPosition(gposdes)
CoMTask.setVelocity(gveldes)


##### OBSERVERS
cpobs = ctrl.add_updater(xic.observers.CoMPositionObserver(ctrl.getModel()))


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import dsimi.interactive
#dsimi.interactive.shell()()
time.sleep(5.)

wm.stopAgents()
ctrl.s.stop()



##### RESULTS
import pylab as pl

cpos = cpobs.get_record()
pl.plot(cpos)
pl.show()


