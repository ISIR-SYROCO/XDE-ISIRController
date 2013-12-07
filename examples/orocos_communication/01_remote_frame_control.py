#!/xde


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import lgsm
import time

import swig_isir_controller as sic
import xde_isir_controller  as xic


import sys
import rtt_interface_corba
rtt_interface_corba.Init(sys.argv)


###### AGENTS
dt = 0.01


###### ROBOT
rname = "kuka"
robotWorld = xrl.createWorldFromUrdfFile(xr.kuka, rname, [0,0,0,1,0,0,0], True, 0.001, 0.01, use_collada_color=False)


dynModelXDE = xrl.getDynamicModelFromWorld(robotWorld)
dynModel    = xic.getModelFromSharedLibrary("../resources/libModelKukaFixed.so", "Create_kukafixed", rname)
#dynModel    = xic.getModelFromXDEDynamicModel(dynModelXDE)


remoteSF = sic.RemoteSegmentFrame("remote.EE.SegmentFrame", dynModel, rname+".07", lgsm.Displacement())

remoteTF = sic.RemoteTargetFrame("remote.EE.TargetFrame", dynModel)

remoteModel = sic.RemoteModel("remote.Model.kuka", dynModel)


rtt_interface_corba.SetServer(remoteSF.getTaskContext())
rtt_interface_corba.SetServer(remoteTF.getTaskContext())
rtt_interface_corba.SetServer(remoteModel.getTaskContext())

# Can be set here, or in the proxy part of the control
#gposdes = lgsm.Displacement(.4,.4,.4,1,0,0,0)
#gveldes = lgsm.Twist()
#TF.setPosition(gposdes)
#TF.setVelocity(gveldes)


import xdefw.interactive
xdefw.interactive.shell_console()()

