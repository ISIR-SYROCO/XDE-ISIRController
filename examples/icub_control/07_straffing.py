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
dynModel = physicshelper.createDynamicModel(robotWorld, rname)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 2, 2.5)
robot.enableContactWithBody("ground.ground", True)
wm.addInteraction([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact


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
ctrl = xic.ISIRCtrl("/home/joe/dev/EReval/orcisir_ISIRController/build/src", dynModel, rname, wm.phy, wm.icsync, "quadprog", True)

ctrl.setTorqueLimits( 80.*lgsm.np.ones(N) )
ctrl.setJointLimitsHorizonOfPrediction(.2)
ctrl.enableJointLimits(False)
robot.setJointPositionsMin(-10*lgsm.ones(N))
robot.setJointPositionsMax(+10*lgsm.ones(N))

##### SET TASKS
fullTask = ctrl.createFullTask("full", 0.0001, kp=9., pos_des=qinit)

#waistTask   = ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "RZ", 10.0, kp=25., pos_des=lgsm.Displacement(0,0,.56,0,0,0,1))

back_name   = [rname+"."+n for n in ['lap_belt_1', 'lap_belt_2', 'chest']]
backTask    = ctrl.createPartialTask("back", back_name, 1.0, kp=25., pos_des=lgsm.zeros(3))


sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

i=0
l_contacts = []
r_contacts = []
for y in [-.027, .027]:
    for z in [-.031, .099]:
        ct = ctrl.createContactTask("CLF"+str(i), rname+".l_foot", lgsm.Displacement(lgsm.vector(-.039, y, z), RotLZdown), 1.5, 0.) # mu, margin
        l_contacts.append(ct)
        ct = ctrl.createContactTask("CRF"+str(i), rname+".r_foot", lgsm.Displacement(lgsm.vector(-.039, y,-z), RotRZdown), 1.5, 0.) # mu, margin
        r_contacts.append(ct)
        i+=1

#for c in l_contacts + r_contacts:
#    c.activateAsConstraint()
#    c.setWeight(10.)


##### SET TASK CONTROLLERS
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
H_lf_sole = lgsm.Displacement(lgsm.vector(-.039, 0, .034), RotLZdown )
H_rf_sole = lgsm.Displacement(lgsm.vector(-.039, 0,-.034), RotRZdown )
walkingTask = xic.walk.WalkingTask( ctrl, dt, 
                                    rname+".l_foot", H_lf_sole, l_contacts,
                                    rname+".r_foot", H_rf_sole, r_contacts,
                                    rname+'.waist', lgsm.Displacement(0,0,0,0,0,0,1), lgsm.Displacement(0,0,.56),
                                    H_0_planeXY=lgsm.Displacement(0,0,0.002), weight=10., contact_as_objective=True)

walkingTask.set_zmp_control_parameters(QonR=1e-6, horizon=1.6, stride=3, gravity=9.81)

walkingTask.stayIdle()


##### OBSERVERS
from observers import ZMPLIPMPositionObserver
zmplipmpobs = ZMPLIPMPositionObserver(dynModel, lgsm.Displacement(0,0,0.002), dt, 9.81, wm.phy, wm.icsync)
zmplipmpobs.s.start()


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import dsimi.interactive
#dsimi.interactive.shell()()
time.sleep(0.)
print "WALK .5m front"


walkingTask.set_step_parameters(length=.05, side=.1, height=.02, time=1, ratio=.9, start_foot="left")
zmp_ref = walkingTask.goTo([0., 0.5], angle=0)


walkingTask.wait_for_end_of_walking()
time.sleep(.5)

wm.stopAgents()
ctrl.s.stop()



##### RESULTS
zmplipmpobs.s.stop()

zmplipmpobs.plot(zmp_ref)


