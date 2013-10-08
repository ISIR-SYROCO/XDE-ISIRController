#!/xde

""" Module to make a proxy of the orcisir_ISIRModel based on the orc framework.

orcisir_ISIRModel can be found in "https://hotline.isir.upmc.fr/wsvn/EReval".
"""


import deploy.deployer as ddeployer
import xdefw.rtt
import physicshelper

from collections import OrderedDict
from time import time
import pylab as pl



class ISIRModel(xdefw.rtt.Task):
    """ Proxy of ISIRModel defined in the module 'XDE-ISIRController-gnulinux'.
    
    This allows to bypass the controller when looking for information on model.
    The model is separated from the controller, gives simpler methods and leads to clearer code.
    
    """

    _NBISIRMODEL = 0

    def __init__(self, libdir, dynamic_model, createFunctionName="Create", robot_name="robot", dt=0.):
        """
        :param ctrl: the controller in which the task has been registered
        :type  ctrl: :class:`ISIRCtrl`
        
        """
        oiname = "oISIRModel"+str(ISIRModel._NBISIRMODEL)
        orocos_IMTask = ddeployer.load(oiname, "XDE_ISIRModel",
                                       module="XDE-ISIRModel-gnulinux", prefix="",
                                       libdir=libdir)
        super(ISIRModel, self).__init__(orocos_IMTask)
        
        ISIRModel._NBISIRMODEL += 1
        
#        set dynamic model
        if isinstance(dynamic_model, physicshelper.DynamicModel):
            self.s.setModelFromXDEPointerStr(str(dynamic_model.this.__long__()), dt)
        elif isinstance(dynamic_model, basestring):
            self.s.setModelFromSharedLibrary(dynamic_model, createFunctionName, robot_name)
        else:
            raise ValueError, "cannot load dynamic_model: "+str(dynamic_model)

    def nbDofs(self):
        return self.s.model_nbDofs()
    def nbInternalDofs(self):
        return self.s.model_nbInternalDofs()
    def hasFixedRoot(self):
        return self.s.model_hasFixedRoot()

    def nbSegments(self):
        return self.s.model_nbSegments()
    def getActuatedDofs(self):
        return self.s.model_getActuatedDofs()
    def getJointLowerLimits(self):
        return self.s.model_getJointLowerLimits()
    def getJointUpperLimits(self):
        return self.s.model_getJointUpperLimits()

    def getJointPositions(self):
        return self.s.model_getJointPositions()
    def getJointVelocities(self):
        return self.s.model_getJointVelocities()
    def getFreeFlyerPosition(self):
        return self.s.model_getFreeFlyerPosition()
    def getFreeFlyerVelocity(self):
        return self.s.model_getFreeFlyerVelocity()

    def getMass(self):
        return self.s.model_getMass()
    def getCoMPosition(self):
        return self.s.model_getCoMPosition()
    def getCoMVelocity(self):
        return self.s.model_getCoMVelocity()
    def getCoMJdotQdot(self):
        return self.s.model_getCoMJdotQdot()
    def getCoMJacobian(self):
        return self.s.model_getCoMJacobian()
    def getCoMJacobianDot(self):
        return self.s.model_getCoMJacobianDot()

    def getInertiaMatrix(self):
        return self.s.model_getInertiaMatrix()
    def getInertiaMatrixInverse(self):
        return self.s.model_getInertiaMatrixInverse()
    def getDampingMatrix(self):
        return self.s.model_getDampingMatrix()
    def getNonLinearTerms(self):
        return self.s.model_getNonLinearTerms()
    def getLinearTerms(self):
        return self.s.model_getLinearTerms()
    def getGravityTerms(self):
        return self.s.model_getGravityTerms()

    def getSegmentPosition(self, index):
        return self.s.model_getSegmentPosition(index)
    def getSegmentVelocity(self, index):
        return self.s.model_getSegmentVelocity(index)
    def getSegmentJacobian(self, index):
        return self.s.model_getSegmentJacobian(index)
    def getSegmentJdot(self, index):
        return self.s.model_getSegmentJdot(index)
    def getSegmentJdotQdot(self, index):
        return self.s.model_getSegmentJdotQdot(index)
    def getJointJacobian(self, index):
        return self.s.model_getJointJacobian(index)
    def getSegmentMass(self, index):
        return self.s.model_getSegmentMass(index)
    def getSegmentCoM(self, index):
        return self.s.model_getSegmentCoM(index)
    def getSegmentMassMatrix(self, index):
        return self.s.model_getSegmentMassMatrix(index)
    def getSegmentMomentsOfInertia(self, index):
        return self.s.model_getSegmentMomentsOfInertia(index)
    def getSegmentInertiaAxes(self, index):
        return self.s.model_getSegmentInertiaAxes(index)


    def getJointDamping(self):
        return self.s.model_getJointDamping()
    def getSegmentIndex(self, name):
        return self.s.model_getSegmentIndex(name)
    def getSegmentName(self, index):
        return self.s.model_getSegmentName(index)

    def setState(self, newq, newdq, newHroot, newTroot):
        self.s.model_setState(newq, newdq, newHroot, newTroot)

    def setStateFromRobot(self, robot):
        newq     = robot.getJointPositions()
        newdq    = robot.getJointVelocities()
        newHroot = robot.getSegmentPosition1(0)     # 0 because root
        newTroot = robot.getSegmentVelocity1(0)     #
        self.s.model_setState(newq, newdq, newHroot, newTroot)




