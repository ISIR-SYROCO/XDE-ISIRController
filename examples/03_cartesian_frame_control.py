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
ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, rname, wm.phy, wm.icsync, "quadprog")


gposdes = 0.5 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", 0.0001)  # create full task with a very low weight for reference posture
fullTask.setKpKd(10)
fullTask.update(gposdes, gveldes)


gposdes = lgsm.Displacement(.4,.4,.4,1,0,0,0)
gveldes = lgsm.Twist()
EETask = ctrl.createFrameTask("EE", "robot.07", lgsm.Displacement(), "RXYZ", 1.) # dofs can be replaced by combination of
EETask.setKpKd(20)
EETask.update(gposdes, gveldes)



##### OBSERVERS
fpobs = ctrl.updater.register(xic.observers.FramePoseObserver(dynModel, "robot.07", lgsm.Displacement()) )


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

fpos = fpobs.get_record()
pl.plot(fpos)
pl.show()


