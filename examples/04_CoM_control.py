#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import physicshelper
import lgsm
import time


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow", 640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()


##### CTRL
import xde_isir_controller as xic
dynModel = physicshelper.createDynamicModel(robotWorld, rname)
ctrl = xic.ISIRCtrl("/home/joe/dev/EReval/XDE-ISIRController/build/src", dynModel, rname, wm.phy, wm.icsync, "quadprog")


gposdes = 0.5 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", 0.0001)  # create full task with a very low weight for reference posture
fullTask.setKpKd(10)
fullTask.update(gposdes, gveldes)


gposdes = lgsm.Displacement(.1,.1,.2,1,0,0,0)
gveldes = lgsm.Twist()
CoMTask = ctrl.createCoMTask("CoM", "XYZ", 1.) # dofs can be replaced by combination of 
CoMTask.setKpKd(20)
CoMTask.update(gposdes, gveldes)


##### OBSERVERS
import observers
jpobs = observers.CoMPositionObserver(dynModel, wm.phy, wm.icsync)
jpobs.s.start()


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
jpobs.s.stop()

jpobs.plot()
