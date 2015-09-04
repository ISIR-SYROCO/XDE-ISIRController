import xde_world_manager as xwm
import xde_resources as xr
import xde_robot_loader as xrl

import xdefw.interactive
shell = xdefw.interactive.shell()

import deploy.deployer as ddeployer

import xde_resources as xr

import xde_robot_loader as xrl

import physicshelper

import lgsm
import time

TIME_STEP = 0.01 

mu_sys = 0.5

wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP)

pi = lgsm.np.pi

import xde_spacemouse as spacemouse

##### GROUND
ground_fixed_base = True
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,-0.8773,1,0,0,0], ground_fixed_base, 0.001, 0.001)
wm.addWorld(groundWorld)

##### ROBOT
rname = "romeo"
robot_fixed_base = False
romeoWorld = xrl.createWorldFromUrdfFile(xr.romeo_collision, rname, [0,0,0.0,1,0,0,0], robot_fixed_base, 0.003, 0.001)

wm.addWorld(romeoWorld)

#Controller
control = xdefw.rtt.Task(ddeployer.load("control", "ISIRControllerThreadXDE", "ISIRControllerThreadXDE-gnulinux", "", libdir="../../_build/"))

#import xde.desc.physic.physic_pb2
#model = xde.desc.physic.physic_pb2.MultiBodyModel()
#model.kinematic_tree.CopyFrom(romeoWorld.scene.physical_scene.nodes[0])
#model.meshes.extend(romeoWorld.library.meshes)
#model.mechanism.CopyFrom(romeoWorld.scene.physical_scene.mechanisms[0])
#model.composites.extend(romeoWorld.scene.physical_scene.collision_scene.meshes)
#dynmodel = physicshelper.createDynamicModel(model)

dynmodel = xrl.getDynamicModelFromWorld(romeoWorld)
romeo = wm.phy.s.GVM.Robot(rname)
romeo.enableGravity(True)

jointmap = xrl.getJointMapping(xr.romeo_collision, romeo)
N  = romeo.getJointSpaceDim()

control.s.setDynModel(str(dynmodel.this.__long__()), rname, str(id(jointmap)), "sequence_Romeo_balance")

#create connectors to get robot k1g state 'k1g_q', 'k1g_qdot', 'k1g_Hroot', 'k1g_Troot', 'k1g_H'
wm.phy.s.Connectors.OConnectorRobotState.new("ocpos"+rname, rname+"_", rname)
wm.phy.s.Connectors.IConnectorRobotJointTorque.new("ict"+rname, rname+"_", rname)

#connector = wm.phy.s.Connectors.OConnectorContactBody.new("robotground", "port_robotground")
#connector.addInteraction("romeo.r_ankle", "ground.ground")
#wm.phy.getPort("port_robotground").connectTo(control.getPort("contacts"))

wm.phy.getPort(rname+"_q").connectTo(control.getPort("q"))
wm.phy.getPort(rname+"_qdot").connectTo(control.getPort("qdot"))
wm.phy.getPort(rname+"_Troot").connectTo(control.getPort("t"))
wm.phy.getPort(rname+"_Hroot").connectTo(control.getPort("d"))
control.getPort("tau").connectTo(wm.phy.getPort(rname+"_tau"))

# Configure the robot
qinit = lgsm.zeros(N)
for name, val in [("LShoulderPitch", pi/2.), ("RShoulderPitch", pi/2.), ("LElbowRoll", -pi/2.), ("RElbowRoll", pi/2.)]:
    qinit[jointmap[rname+"."+name]] = val

##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, mu_sys)
romeo.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_ankle", "r_ankle"]])

##### SET INITIAL CONFIGURATION
romeo.setJointPositions(qinit)
dynmodel.setJointPositions(qinit)
romeo.setJointVelocities(lgsm.zeros(N))
dynmodel.setJointVelocities(lgsm.zeros(N))

#control.s.setPeriod(TIME_STEP)
control.s.setTimeStep(TIME_STEP)
control.s.start()

#PDC Control mode
#sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=True, body_name="k1g.07")

# Normal mode
#sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=False)

#sm.s.start()

wm.startAgents()
