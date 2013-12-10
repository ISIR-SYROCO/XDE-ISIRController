
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


