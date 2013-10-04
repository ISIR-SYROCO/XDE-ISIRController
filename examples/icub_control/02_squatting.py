#!/xde

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import physicshelper
import lgsm
import time

pi = lgsm.np.pi





def get_sin_traj(x0, A, T, phi, t0, tend, time_step):
    traj = []
    omega = 2.*lgsm.np.pi/T
    timeline = lgsm.np.arange(t0, tend, time_step)
    for t in timeline:
        x = x0            + A * lgsm.np.sin(omega*t+phi)
        v =    + omega    * A * lgsm.np.cos(omega*t+phi)
        a =    - omega**2 * A * lgsm.np.sin(omega*t+phi)
        traj.append( (x, v, a) )
    return timeline, traj





##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### GROUND
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)


##### ROBOT
rname = "robot"
fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, rname, [0,0,0.6,1,0,0,0], fixed_base, 0.001, 0.001)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N  = robot.getJointSpaceDim()

import xde.desc.physic
multiBodyModel = xde.desc.physic.physic_pb2.MultiBodyModel()
multiBodyModel.kinematic_tree.CopyFrom(robotWorld.scene.physical_scene.nodes[0])
multiBodyModel.meshes.extend(robotWorld.library.meshes)
multiBodyModel.mechanism.CopyFrom(robotWorld.scene.physical_scene.mechanisms[0])
multiBodyModel.composites.extend(robotWorld.scene.physical_scene.collision_scene.meshes)
dynModel = physicshelper.createDynamicModel(multiBodyModel)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact


##### SET INITIAL STATE
qinit = lgsm.zeros(N)
# correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch      l_shoulder_roll          r_shoulder_roll
for name, val in [("l_arm", pi/8.), ("r_arm", pi/8.), ("l_thigh", -0.05), ("r_thigh", -0.05), ("l_shank", -0.05), ("r_shank", -0.05), ("l_shoulder_1", pi/8.), ("r_shoulder_1", pi/8.)]:
    qinit[robot.getSegmentIndex(rname+"."+name)] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


##### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, rname, wm.phy, wm.icsync, "quadprog", True)


##### SET TASKS
# this ...
#N0 = 6 if fixed_base is False else 0
#partialTask = ctrl.createPartialTask("partial", range(N0, N+N0), 0.0001, kp=9., pos_des=qinit)
# ... is equivalent to that:
fullTask = ctrl.createFullTask("full", 0.0001, kp=9., pos_des=qinit)

waistTask   = ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "RXYZ", 1., kp=36., pos_des=lgsm.Displacement(0,0,.58,1,0,0,0))


back_name   = [rname+"."+n for n in ['lap_belt_1', 'lap_belt_2', 'chest']]
backTask    = ctrl.createPartialTask("back", back_name, 0.001, kp=16., pos_des=lgsm.zeros(3))


sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)


ctrl.createContactTask("CLF0", rname+".l_foot", lgsm.Displacement([-.039,-.027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF1", rname+".l_foot", lgsm.Displacement([-.039, .027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF2", rname+".l_foot", lgsm.Displacement([-.039, .027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF3", rname+".l_foot", lgsm.Displacement([-.039,-.027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin

ctrl.createContactTask("CRF0", rname+".r_foot", lgsm.Displacement([-.039,-.027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF1", rname+".r_foot", lgsm.Displacement([-.039, .027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF2", rname+".r_foot", lgsm.Displacement([-.039, .027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF3", rname+".r_foot", lgsm.Displacement([-.039,-.027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin



##### SET TASK CONTROLLERS
ref_timeline, trajz = get_sin_traj(.55, .02, 5., 0, 0, 30., dt)
waistTraj = [ (lgsm.Displacement(0,0,z,1,0,0,0), lgsm.Twist(lgsm.vector(0,0,0,0,0,vz)), lgsm.Twist(lgsm.vector(0,0,0,0,0,az))) for z,vz,az in trajz]

ctrl.task_updater.register( xic.task_controller.TrajectoryTracking(waistTask, waistTraj) )


##### OBSERVERS
from observers import FramePoseObserver
fpobs = FramePoseObserver(robot, rname+'.waist', lgsm.Displacement(), wm.phy, wm.icsync)
fpobs.s.start()

##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(30.)

wm.stopAgents()
ctrl.s.stop()


##### RESULTS
fpobs.s.stop()

fpobs.plot([z for z, vz, az in trajz])
