#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time


class RemoteModelUpdater:
    def __init__(self, proxy_model):
        self.proxy_model = proxy_model

    def update(self):
        self.proxy_model.updateModel()


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow", 640, 480, 1000, 50)


##### ROBOT
rname = "kuka"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01, use_collada_color=False)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N = robot.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(robotWorld)


##### CTRL
import xde_isir_controller as xic
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "quadprog", False)


gposdes = 0.5 * lgsm.ones(N)
gveldes = lgsm.zeros(N)
fullTask = ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=10.)  # create full task with a very low weight for reference posture
fullTask.set_q(gposdes)
fullTask.set_qdot(gveldes)




import swig_isir_controller as sic

import xde_isir_controller.corba as corba

CoMF      = sic.ProxyCoMFrame("proxy.CoM.CoMFrame", corba.getProxyTaskContext("remote.CoM.CoMFrame")._obj)

TF       = sic.ProxyTargetFrame("proxy.CoM.TargetFrame", corba.getProxyTaskContext("remote.CoM.TargetFrame")._obj)

proxyModel  = sic.ProxyModel(ctrl.getModel(), corba.getProxyTaskContext("remote.Model.kuka")._obj)


feat     = sic.PositionFeature("CoM.PositionFeature"    , CoMF, sic.XYZ)
featDes  = sic.PositionFeature("CoM.PositionFeature_Des", TF  , sic.XYZ)

CoMTask   = ctrl.createGenericTask("CoM", "acceleration", CoMF, feat, TF, featDes, w=1., kp=20.)

gposdes = lgsm.Displacement(.1,.1,.2,1,0,0,0)
gveldes = lgsm.Twist()
CoMTask.setPosition(gposdes)
CoMTask.setVelocity(gveldes)

##### OBSERVERS
cpobs = ctrl.add_updater(xic.observers.CoMPositionObserver(ctrl.getModel()))

ctrl.add_updater(RemoteModelUpdater(proxyModel) )

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
import pylab as pl

cpos = cpobs.get_record()
pl.plot(cpos)
pl.show()


