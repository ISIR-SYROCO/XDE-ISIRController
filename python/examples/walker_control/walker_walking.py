#!/xde

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time

pi = lgsm.np.pi

##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 200, 50)


##### GROUND
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)


##### ROBOT
rname = "robot"
fixed_base = False
#robotWorld = xrl.createWorldFromUrdfFile("urdf/walker.xml", rname, [0,0,0.95,1,0,0,0], fixed_base, 0.001, 0.001)
robotWorld = xrl.createWorldFromUrdfFile("urdf/walker.xml", rname, [0,0,0.95,0.707,0,0,0.707], fixed_base, 0.001, 0.001)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N  = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


###### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact


###### SET INITIAL STATE
qinit = lgsm.zeros(N)
for name, val in [("l_thigh_roll", -0.2), ("r_thigh_roll", -0.2), ("l_thigh", 0.4), ("r_thigh", 0.4), ("l_foot_roll", -0.2), ("r_foot_roll", -0.2)]:
    qinit[robot.getSegmentIndex(rname+"."+name)] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


###### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog", False)


###### SET TASKS
fullTask = ctrl.createFullTask("full", w=0.0001, kp=9., q_des=qinit)

i=0
l_contacts = []
r_contacts = []
RotZdown = lgsm.Quaternion(0,1,0,0)
for x in [-.1, .2]:
    for y in [-.075, .075]:
        ct = ctrl.createContactTask("CLF"+str(i), rname+".l_foot", lgsm.Displacement([x,y, -0.005]+RotZdown.tolist()), 1.5)
        l_contacts.append(ct)
        ct = ctrl.createContactTask("CRF"+str(i), rname+".r_foot", lgsm.Displacement([x,y, -0.005]+RotZdown.tolist()), 1.5)
        r_contacts.append(ct)
        i+=1

#for c in l_contacts + r_contacts:
#    c.activateAsConstraint()

for c in l_contacts + r_contacts:
    c.deactivate()

for c in l_contacts + r_contacts:
        c.activateAsObjective()

print "===================================================================================="

##### SET TASK CONTROLLERS
RotZUp = lgsm.Quaternion(1,0,0,0)
H_lf_sole = lgsm.Displacement([0.05, -0.05, -0.005]+RotZUp.tolist() )
H_rf_sole = lgsm.Displacement([0.05,  0.05, -0.005]+RotZUp.tolist() )
walkingTask = xic.walk.WalkingActivity( ctrl, dt, 
                                    rname+".l_foot", H_lf_sole, l_contacts,
                                    rname+".r_foot", H_rf_sole, r_contacts,
                                    rname+'.waist', lgsm.Displacement(), lgsm.Displacement(0,0,.7,1,0,0,0),
                                    H_0_planeXY=lgsm.Displacement(0,0,0.000,1,0,0,0), weight=10., contact_as_objective=True)

#walkingTask.stayIdle(com_position=[.1, .15])

walkingTask.set_step_parameters(length=0.2, side=.05, height=.02, time=0.8, ratio=.9, start_foot="left")
zmp_ref = walkingTask.goTo([0,5.])

###### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

##import dsimi.interactive
##dsimi.interactive.shell()()

#time.sleep(1.1)

#for i in range(10):
#    time.sleep(.1)

#    for c in r_contacts:
#        c.deactivate()

#    time.sleep(.1)

#    for c in r_contacts:
#        c.activateAsObjective()

#    time.sleep(.1)

walkingTask.wait_for_end_of_walking()
print "END OF WALKING TASK"

wm.stopAgents()
ctrl.s.stop()


import xdefw.interactive
xdefw.interactive.shell_console()()


