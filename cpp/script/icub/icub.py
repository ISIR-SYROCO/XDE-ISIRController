import xde_world_manager as xwm
import xde_resources as xr
import xde_robot_loader as xrl

import xdefw.interactive
shell = xdefw.interactive.shell()

import deploy.deployer as ddeployer
import xde_resources as xr
import xde_robot_loader as xrl
import xde_spacemouse as spacemouse
import physicshelper
import lgsm
import time
import math

TIME_STEP = 0.1 

M_SQRT1_2 = 1/math.sqrt(2)

wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP)

pi = lgsm.np.pi
mu_sys = 0.5


##### GROUND
ground_fixed_base = True
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], ground_fixed_base, 0.001, 0.001)
wm.addWorld(groundWorld)

##### ROBOT
rname = "icub"
robot_fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, rname, [0,0,0.6,-M_SQRT1_2,0,0,M_SQRT1_2], robot_fixed_base, 0.001, 0.001)

wm.addWorld(robotWorld)

#Controller
ctrl = xdefw.rtt.Task(ddeployer.load("control", "ISIRControllerThreadXDE", "ISIRControllerThreadXDE-gnulinux", "", libdir="../../_build/"))

dynModel = xrl.getDynamicModelFromWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

jointmap = xrl.getJointMapping(xr.icub_simple, robot)

#sequence = "sequence_iCub_02_squatting"
sequence = "sequence_iCub_03_dbal_standing"
ctrl.s.setDynModel(str(dynModel.this.__long__()), rname, str(id(jointmap)), sequence)

wm.phy.s.Connectors.OConnectorRobotState.new("ocpos"+rname, rname+"_", rname)
wm.phy.s.Connectors.IConnectorRobotJointTorque.new("ict"+rname, rname+"_", rname)
wm.icsync.addEvent(rname+"_tau")

wm.phy.getPort(rname+"_q").connectTo(ctrl.getPort("q"))
wm.phy.getPort(rname+"_qdot").connectTo(ctrl.getPort("qdot"))
wm.phy.getPort(rname+"_Troot").connectTo(ctrl.getPort("t"))
wm.phy.getPort(rname+"_Hroot").connectTo(ctrl.getPort("d"))
ctrl.getPort("tau").connectTo(wm.phy.getPort(rname+"_tau"))

##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, mu_sys)
robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact


##### SET INITIAL STATE
#qinit = lgsm.zeros(N)
#for name, val in [("l_shoulder_pitch", pi/8.),("r_shoulder_pitch", pi/8.),("l_elbow_pitch", pi/8.), ("r_elbow_pitch", pi/8.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05),("r_ankle_pitch", -0.05)]:
#    qinit[jointmap[rname+"."+name]] = val

qinit = lgsm.zeros(N)
for name, val in [("l_elbow_pitch", pi/8.), ("r_elbow_pitch", pi/8.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", pi/8.), ("r_shoulder_roll", pi/8.)]:
    qinit[jointmap[rname+"."+name]] = val

robot.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointPositions(qinit)
dynModel.setJointVelocities(lgsm.zeros(N))

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

##### SIMULATE
ctrl.s.setTimeStep(TIME_STEP)
ctrl.s.start()


#import xdefw.interactive
#xdefw.interactive.shell_console()()
#time.sleep(30.)

#wm.stopAgents()
#ctrl.s.stop()

