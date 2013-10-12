#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.01)
wm.resizeWindow("mainWindow", 640, 480, 1000, 50)


##### ROBOT
robotWorld = xrl.createWorldFromUrdfFile("resources/moving_wall.xml", "moving_wall", [0.6,0,0.25,1,0,0,0], True, 0.001, 0.01)
wm.addWorld(robotWorld)

dynModel_moving_wall = xrl.getDynamicModelFromWorld(robotWorld)



rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


#import xdefw.interactive
#xdefw.interactive.shell_console()()

##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.metal", 1, 1.5)
robot.enableContactWithBody("moving_wall.moving_wall", True)
wm.contact.showContacts([(b,"moving_wall.moving_wall") for b in robot.getSegmentNames()]) # to display contact


##### INIT ROBOT & MODEL
qinit = lgsm.vector([0,0,0,-lgsm.math.pi/2.,0,0,0])

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


##### CTRL
import xde_isir_controller as xic
ctrl_moving_wall = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel_moving_wall, "moving_wall", wm.phy, wm.icsync, "quadprog")

gposdes = lgsm.zeros(1)
gveldes = lgsm.zeros(1)
fullTask = ctrl_moving_wall.createFullTask("zero_wall", 1.)  # create full task with a very low weight for reference posture
fullTask.setKpKd(20)
fullTask.update(gposdes, gveldes)



ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, rname, wm.phy, wm.icsync, "quadprog")


#gposdes = 0.5 * lgsm.ones(N)
gposdes = qinit
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", 0.0001)  # create full task with a very low weight for reference posture
fullTask.setKpKd(0, 10)
fullTask.update(gposdes, gveldes)

##### To do an impedance task (an acceleration task without contact)
#gposdes = lgsm.Displacement(.8,0,.25,1,0,0,0)
#gveldes = lgsm.Twist()
#EETask = ctrl.createFrameTask("EE", "robot.07", lgsm.Displacement(), "XYZ", 1.)
#EETask.setKpKd(10)
#EETask.update(gposdes, gveldes)

##### To do a real force task
EETask = ctrl.createForceTask("EE", "robot.07", lgsm.Displacement(), 1.0)
EETask.update(lgsm.vector(10,0,0), True)

##### OBSERVERS
tpobs = ctrl.updater.register(xic.observers.TorqueObserver(ctrl_moving_wall))

##### SIMULATE
ctrl.s.start()
ctrl_moving_wall.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(5.)

wm.stopAgents()
ctrl.s.stop()
ctrl_moving_wall.s.stop()


##### RESULTS
import pylab as pl

tpos = tpobs.get_record()
pl.plot(tpos)

pl.show()


