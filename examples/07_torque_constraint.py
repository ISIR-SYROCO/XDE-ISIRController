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
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog", True)

torqueConst = ctrl.add_constraint(xic.TorqueLimitConstraint(ctrl.getModel(), lgsm.ones(N)*0.08))

ctrl.controller.takeIntoAccountGravity(False)
robot.enableGravity(False)

torqueTask = ctrl.createTorqueTask("torque", [1], w=1.)


torqueTraj = .1 * lgsm.np.sin(lgsm.np.arange(0,5., dt) * lgsm.np.pi ).reshape((-1,1))
ctrl.add_updater( xic.task_controller.TrajectoryTracking(torqueTask, torqueTraj) )


##### OBSERVERS
jpobs = ctrl.add_updater(xic.observers.JointPositionsObserver(ctrl.getModel()))
tpobs = ctrl.add_updater(xic.observers.TorqueObserver(ctrl))


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(5.)

wm.stopAgents()
ctrl.s.stop()


##### RESULTS
import pylab as pl

jpos = jpobs.get_record()
pl.figure()
pl.plot(jpos)

tpos = tpobs.get_record()
pl.figure()
pl.plot(tpos)

pl.show()


