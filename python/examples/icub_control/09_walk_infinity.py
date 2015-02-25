#!/xde

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time

pi = lgsm.np.pi
np = lgsm.np

def get_infinity_traj(start, direction, R=.5, eps=360, go_left=True):
    """
    """
    Xvec = lgsm.vector(1,0)
    sinT = lgsm.crossprod(Xvec, direction)[0,0]
    cosT = lgsm.dotprod(Xvec  , direction)
    start_angle = lgsm.np.arctan2(sinT, cosT)
    ortho_direction = lgsm.vector(-direction[1,0], direction[0,0])

    left_center  = start + ortho_direction * R
    right_center = start - ortho_direction * R

    traj = []

    if go_left:
        for T in np.linspace(0, 2.*pi, eps):
            a = start_angle + T
            x = left_center[0,0] +R*np.cos(a - pi/2.)
            y = left_center[1,0] +R*np.sin(a - pi/2.)
            traj.append([x,y,a])

        for T in np.linspace(0, 2.*pi, eps):
            a = start_angle - T
            x = right_center[0,0] +R*np.cos(a + pi/2.)
            y = right_center[1,0] +R*np.sin(a + pi/2.)
            traj.append([x,y,a])

    else:
        for T in np.linspace(0, 2.*pi, eps):
            a = start_angle - T
            x = right_center[0,0] +R*np.cos(a + pi/2.)
            y = right_center[1,0] +R*np.sin(a + pi/2.)
            traj.append([x,y,a])

        for T in np.linspace(0, 2.*pi, eps):
            a = start_angle + T
            x = left_center[0,0] +R*np.cos(a - pi/2.)
            y = left_center[1,0] +R*np.sin(a - pi/2.)
            traj.append([x,y,a])


    return np.array(traj)


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.01, uc_relaxation_factor=0.001)
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

dynModel = xrl.getDynamicModelFromWorld(robotWorld)
jmap     = xrl.getJointMapping(xr.icub_simple, robot)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 2, 2.5)
robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact


##### SET INITIAL STATE
qinit = lgsm.zeros(N)
for name, val in [("l_elbow_pitch", pi/8.), ("r_elbow_pitch", pi/8.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", pi/8.), ("r_shoulder_roll", pi/8.)]:
    qinit[jmap[rname+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


robot.setJointPositionsMin(-10*lgsm.ones(N))
robot.setJointPositionsMax(+10*lgsm.ones(N))


##### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", True)

torqueConst = ctrl.add_constraint(xic.TorqueLimitConstraint(ctrl.getModel(), 80.*lgsm.ones(N) ) )
#jointConst  = ctrl.add_constraint(xic.JointLimitConstraint(ctrl.getModel(), .2 ) )

##### SET TASKS
fullTask = ctrl.createFullTask("full", w=0.0001, kp=9., q_des=qinit)

#waistTask   = ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "RZ", w=10.0, kp=25., pose_des=lgsm.Displacement(0,0,.58,0,0,0,1))

back_dofs   = [jmap[rname+"."+n] for n in ['torso_pitch', 'torso_roll', 'torso_yaw']]
backTask    = ctrl.createPartialTask("back", back_dofs, w=1.0, kp=25., q_des=lgsm.zeros(3))


sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

i=0
l_contacts = []
r_contacts = []
for y in [-.027, .027]:
    for z in [-.031, .099]:
        ct = ctrl.createContactTask("CLF"+str(i), rname+".l_foot", lgsm.Displacement([-.039, y, z]+RotLZdown.tolist()), 1.5)
        l_contacts.append(ct)
        ct = ctrl.createContactTask("CRF"+str(i), rname+".r_foot", lgsm.Displacement([-.039, y,-z]+RotRZdown.tolist()), 1.5)
        r_contacts.append(ct)
        i+=1

#for c in l_contacts + r_contacts:
#    c.activateAsConstraint()
#    c.setWeight(10.)


##### SET TASK CONTROLLERS
RotLZUp = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
RotRZUp = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,0.0,0.0,1.0)
H_lf_sole = lgsm.Displacement([-.039, 0, .034]+RotLZUp.tolist() )
H_rf_sole = lgsm.Displacement([-.039, 0,-.034]+RotRZUp.tolist() )
walkingActivity = xic.walk.WalkingActivity( ctrl, dt,
                                    rname+".l_foot", H_lf_sole, l_contacts,
                                    rname+".r_foot", H_rf_sole, r_contacts,
                                    rname+'.waist', lgsm.Displacement(0,0,0,0,0,0,1), lgsm.Displacement(0,0,.58,1,0,0,0),
                                    H_0_planeXY=lgsm.Displacement(0,0,0.002,1,0,0,0), weight=10., contact_as_objective=True)

#walkingActivity.stayIdle()



inf_traj = get_infinity_traj(lgsm.vector(0.,0.), lgsm.vector(1,0), R=.5, go_left=True)
zmp_ref = walkingActivity.followTrajectory(inf_traj)

##### OBSERVERS
zmplipmpobs = ctrl.add_updater( xic.observers.ZMPLIPMPositionObserver(ctrl.getModel(), lgsm.Displacement(), dt, 9.81) )


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()



walkingActivity.wait_for_end_of_walking()
time.sleep(2.)


wm.stopAgents()
ctrl.s.stop()



##### RESULTS
import pylab as pl
zmplipm = zmplipmpobs.get_record()
pl.plot(zmplipm)
pl.plot(zmp_ref, ls=":")
pl.show()


