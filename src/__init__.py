
# WARNING:
# The load of this physic component should be done before loading
# 'swig_isir_controller', because if not, getModelFromXDEDynamicModel will
# proceed erroneously. Maybe not the best solution, but at least, it works...
import agents.physic.core
agents.physic.core.createAgent("_fake_physic_agent_to_load_model_from_xde_", 0)

# import some important elements from swig_isir_controller ...
import swig_isir_controller

# ... from models
from swig_isir_controller import getModelFromXDEDynamicModel, getModelFromSharedLibrary

# ... from solvers
from swig_isir_controller import OneLevelSolverWithQuadProg, OneLevelSolverWithQLD

# ... from constraints
from swig_isir_controller import TorqueLimitConstraint, JointLimitConstraint, ContactAvoidanceConstraint
from contact              import ContactAvoidanceConstraintUpdater



#####
from core  import ISIRController, ISIRTask

import observers
import task_controller
import walk
import performances

try:
    from swig_isir_controller import TaskXMLParser
    fullstateCast = swig_isir_controller.isir_task_manager.fullstateCast
    partialstateCast = swig_isir_controller.isir_task_manager.partialstateCast
    frameCast = swig_isir_controller.isir_task_manager.frameCast
    displacementCast = swig_isir_controller.isir_task_manager.displacementCast
    positionCast = swig_isir_controller.isir_task_manager.positionCast
    orientationCast = swig_isir_controller.isir_task_manager.orientationCast
    comCast = swig_isir_controller.isir_task_manager.comCast
    contactCast = swig_isir_controller.isir_task_manager.contactCast
    trajectoryReaderFullJointCast = swig_isir_controller.isir_task_manager.trajectoryReaderFullJointCast
except:
    TaskXMLParser = None

