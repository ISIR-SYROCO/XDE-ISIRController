#!/xde

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import physicshelper
import lgsm
import time

pi = lgsm.np.pi


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.01, uc_relaxation_factor=0.01)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### GROUND
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)


##### ROBOT
rname = "robot"
fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, rname, [0,0,0.6,0,0,0,1], fixed_base, .003, 0.001)
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
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 2, 2.5)
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
ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, rname, wm.phy, wm.icsync, "qld", True)

ctrl.setTorqueLimits( 80.*lgsm.np.ones(N) )
ctrl.setJointLimitsHorizonOfPrediction(.2)
ctrl.enableJointLimits(True)

##### SET TASKS
fullTask = ctrl.createFullTask("full", 0.0001, kp=9., pos_des=qinit)

#waistTask   = ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(0,0,0,0,0,0,1), "RZ", 10.0, kp=25., pos_des=lgsm.Displacement(0,0,.58))

back_name   = [rname+"."+n for n in ['lap_belt_1', 'lap_belt_2', 'chest']]
backTask    = ctrl.createPartialTask("back", back_name, 0.01, kp=25., pos_des=lgsm.zeros(3))


sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

i=0
l_contacts = []
r_contacts = []
for y in [-.027, .027]:
    for z in [-.031, .099]:
        ct = ctrl.createContactTask("CLF"+str(i), rname+".l_foot", lgsm.Displacement([-.039, y, z]+RotLZdown.tolist()), 1.5, 0.) # mu, margin
        l_contacts.append(ct)
        ct = ctrl.createContactTask("CRF"+str(i), rname+".r_foot", lgsm.Displacement([-.039, y,-z]+RotRZdown.tolist()), 1.5, 0.) # mu, margin
        r_contacts.append(ct)
        i+=1

#for c in l_contacts + r_contacts:
#    c.activateAsConstraint()
#    c.setWeight(10.)


##### SET TASK CONTROLLERS
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
H_lf_sole = lgsm.Displacement([-.039, 0, .034]+RotLZdown.tolist() )
H_rf_sole = lgsm.Displacement([-.039, 0,-.034]+RotRZdown.tolist() )
walkingActivity = xic.walk.WalkingActivity( ctrl, dt,
                                    rname+".l_foot", H_lf_sole, l_contacts,
                                    rname+".r_foot", H_rf_sole, r_contacts,
                                    rname+'.waist', lgsm.Displacement(0,0,0,0,0,0,1), lgsm.Displacement(0,0,.58,1,0,0,0),
                                    H_0_planeXY=lgsm.Displacement(0,0,0.002,1,0,0,0), weight=10., contact_as_objective=True)

walkingActivity.set_zmp_control_parameters(RonQ=1e-6, horizon=1.6, stride=3, gravity=9.81, height_ref=0.58, updatePxPu=1e-3, use_swig_zmpy=True)
#walkingTask.stayIdle()


zmp_ref = walkingActivity.goTo([.5,0.])

##### OBSERVERS
from observers import ZMPLIPMPositionObserver
zmplipmpobs = ZMPLIPMPositionObserver(dynModel, lgsm.Displacement(0,0,0.002,1,0,0,0), dt, 9.81, wm.phy, wm.icsync)
zmplipmpobs.s.start()


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()


walkingActivity.wait_for_end_of_walking()
print "END OF WALKING TASK"

wm.stopAgents()
ctrl.s.stop()

perfs = ctrl.getPerformances()

##### RESULTS
zmplipmpobs.s.stop()

#zmplipmpobs.plot(zmp_ref)

xic.performances.plot_performances(perfs)

