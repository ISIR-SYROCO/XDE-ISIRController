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
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### ROBOT
rname = "robot"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,0,1,0,0], True, 0.001, 0.01)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

import xde.desc.physic
multiBodyModel = xde.desc.physic.physic_pb2.MultiBodyModel()
multiBodyModel.kinematic_tree.CopyFrom(robotWorld.scene.physical_scene.nodes[0])
multiBodyModel.meshes.extend(robotWorld.library.meshes)
multiBodyModel.mechanism.CopyFrom(robotWorld.scene.physical_scene.mechanisms[0])
multiBodyModel.composites.extend(robotWorld.scene.physical_scene.collision_scene.meshes)
dynModel = physicshelper.createDynamicModel(multiBodyModel)


##### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, rname, wm.phy, wm.icsync, "quadprog", True)


gposdes = 1.2 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
#fullTask = ctrl.createFullTask("full", 0.001)
#fullTask.setKpKd(20)
#fullTask.update(gposdes, gveldes)

torqueTask = ctrl.createTorqueTask("torque", [1], 1., torque_des=lgsm.vector([0.11]) )


##### OBSERVERS
import observers
jpobs = observers.JointPositionsObserver(robot, wm.phy, wm.icsync)
tpobs = observers.TorqueObserver(ctrl, wm.phy, wm.icsync)
jpobs.s.start()
tpobs.s.start()


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
tpobs.s.stop()
jpobs.s.stop()
tpobs.plot()
jpobs.plot()
