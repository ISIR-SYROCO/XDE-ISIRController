#include "orcXdeModel.h"

#include <map>
#include <vector>
#include <iostream>

//=================================  Class methods  =================================//
orcXdeModel::orcXdeModel(xde::gvm::extra::DynamicModel* xdeModel, std::string rname, PyDictObject* jointList)
    :orcisir::ISIRModel(rname, xdeModel->nbDofs(), !xdeModel->hasFixedRoot())
    , _m(xdeModel)
    , _actuatedDofs(xdeModel->nbInternalDofs())
    , _linearTerms(xdeModel->nbDofs())
    , _jointList(jointList)
{
    _actuatedDofs.setOnes();
    _linearTerms.setZero();

    _segmentName.resize(nbSegments());
    for (int i=0; i<nbSegments(); ++i)
    {
        _segmentName[i] = _m->getSegmentName(i);
    }
}

orcXdeModel::~orcXdeModel()
{
    
}

int orcXdeModel::nbSegments() const
{
    return _m->nbSegments();
}

const Eigen::VectorXd& orcXdeModel::getActuatedDofs() const
{
    return _actuatedDofs;
}

const Eigen::VectorXd& orcXdeModel::getJointLowerLimits() const
{
    return _m->getJointLowerLimits();
}

const Eigen::VectorXd& orcXdeModel::getJointUpperLimits() const
{
    return _m->getJointUpperLimits();
}

const Eigen::VectorXd& orcXdeModel::getJointPositions() const
{
    return _m->getJointPositions();
}

const Eigen::VectorXd& orcXdeModel::getJointVelocities() const
{
    return _m->getJointVelocities(); 
}

const Eigen::Displacementd& orcXdeModel::getFreeFlyerPosition() const
{
    return _m->getFreeFlyerPosition();
}

const Eigen::Twistd& orcXdeModel::getFreeFlyerVelocity() const
{
    return _m->getFreeFlyerVelocity();
}

const Eigen::MatrixXd& orcXdeModel::getInertiaMatrix() const
{
    return _m->getInertiaMatrix();
}

const Eigen::MatrixXd& orcXdeModel::getInertiaMatrixInverse() const
{
    return _m->getInertiaMatrixInverse();
}

const Eigen::MatrixXd& orcXdeModel::getDampingMatrix() const
{
    return _m->getDampingMatrix();
}

const Eigen::VectorXd& orcXdeModel::getNonLinearTerms() const
{
    return _m->getNonLinearTerms();
}

const Eigen::VectorXd& orcXdeModel::getLinearTerms() const
{
    return _linearTerms;
}

const Eigen::VectorXd& orcXdeModel::getGravityTerms() const
{ 
    return _m->getGravityTerms();
}

double orcXdeModel::getMass() const
{
    return _m->getMass();
}

const Eigen::Vector3d& orcXdeModel::getCoMPosition() const
{
    return _m->getCoMPosition();
}

const Eigen::Vector3d& orcXdeModel::getCoMVelocity() const
{
    return _m->getCoMVelocity();
}

const Eigen::Vector3d& orcXdeModel::getCoMJdotQdot() const
{
    return _m->getCoMJdotQdot();
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& orcXdeModel::getCoMJacobian() const
{
    return _m->getCoMJacobian();
}

const Eigen::Matrix<double,COM_POS_DIM,Eigen::Dynamic>& orcXdeModel::getCoMJacobianDot() const
{
    throw std::runtime_error("XdeModel::getCoMJacobianDot not implemented.");
}

const Eigen::Displacementd& orcXdeModel::getSegmentPosition(int index) const
{
    return _m->getSegmentPosition(index);
}

const Eigen::Twistd& orcXdeModel::getSegmentVelocity(int index) const
{
    return _m->getSegmentVelocity(index);
}

double orcXdeModel::getSegmentMass(int index) const
{
    return _m->getSegmentMass(index);
}

const Eigen::Vector3d& orcXdeModel::getSegmentCoM(int index) const
{
    return _m->getSegmentCoM(index);
}

const Eigen::Matrix<double,6,6>& orcXdeModel::getSegmentMassMatrix(int index) const
{
    return _m->getSegmentMassMatrix(index);
}

const Eigen::Vector3d& orcXdeModel::getSegmentMomentsOfInertia(int index) const
{
    return _m->getSegmentMomentsOfInertia(index);
}

const Eigen::Rotation3d& orcXdeModel::getSegmentInertiaAxes(int index) const
{
    return _m->getSegmentInertiaAxes(index);
}

//compute jacobian in segment frame
const Eigen::Matrix<double,6,Eigen::Dynamic>& orcXdeModel::getSegmentJacobian(int index) const
{
    return _m->getSegmentJacobian(index);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcXdeModel::getSegmentJdot(int index) const
{
    throw std::runtime_error("XdeModel::getSegmentJdot not implemented.");
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& orcXdeModel::getJointJacobian(int index) const
{
    return _m->getJointJacobian(index);
}

const Eigen::Twistd& orcXdeModel::getSegmentJdotQdot(int index) const
{
    return _m->getSegmentJdotQdot(index);
}

/*
void XdeModel::wbiSetState(const wbi::Frame& H_root, const Eigen::VectorXd& q, const Eigen::Twistd& T_root, const Eigen::VectorXd& q_dot)
{
    Eigen::Displacementd H;
    Eigen::Twistd T;
    // WBI versions
    owm_pimpl->Hroot_wbi = H_root;
    owm_pimpl->Troot_wbi = T_root;

    // ORC versions
    orcWbiConversions::wbiFrameToEigenDispd(owm_pimpl->Hroot_wbi, H); 
    orcWbiConversions::wbiToOrcTwistVector(owm_pimpl->Troot_wbi, T); 

    setJointPositions(q);
    setJointVelocities(q_dot);
    setFreeFlyerPosition(H);
    setFreeFlyerVelocity(T);

}
*/

void orcXdeModel::doSetJointPositions(const Eigen::VectorXd& q)
{
    _m->setJointPositions(q);
}

void orcXdeModel::doSetJointVelocities(const Eigen::VectorXd& dq)
{ 
    _m->setJointVelocities(dq);
}

void orcXdeModel::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
    _m->setFreeFlyerPosition(Hroot);
}

void orcXdeModel::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
    _m->setFreeFlyerVelocity(Troot);
}

int orcXdeModel::doGetSegmentIndex(const std::string& name) const
{
    return _m->getSegmentIndex(name);
}

const std::string& orcXdeModel::doGetSegmentName(int index) const
{
    return _segmentName[index];
}

int orcXdeModel::doGetDofIndex(const std::string& name) const
{
    return (int)PyInt_AsLong(PyDict_GetItemString((PyObject *)_jointList, this->DofName(name).c_str()));
}

const std::string& orcXdeModel::doGetDofName(int index) const
{
    throw std::runtime_error("[orcXdeModel::doGetDofName] This function was not overriden for a specific model");
}

const std::string orcXdeModel::doSegmentName(const std::string& name) const
{
    // Return robotName.segmentName
    return this->getName() + "." + name;
}

const std::string orcXdeModel::doDofName(const std::string& name) const
{
    // Return robotName.dofName
    return this->getName() + "." + name;
}

void orcXdeModel::printAllData()
{
    std::cout<<"nbSegments:\n";
    std::cout<<nbSegments()<<"\n";

    std::cout<<"nbDofs:\n";
    std::cout<<nbDofs()<<std::endl;
    
    std::cout<<"nbInternalDofs:\n";
    std::cout<<nbInternalDofs()<<std::endl;

    std::cout<<"actuatedDofs:\n";
    std::cout<<getActuatedDofs()<<"\n";
    
    std::cout<<"lowerLimits:\n";
    std::cout<<getJointLowerLimits()<<"\n";
    
    std::cout<<"upperLimits:\n";
    std::cout<<getJointUpperLimits()<<"\n";
    
    std::cout<<"q:\n";
    std::cout<<getJointPositions().transpose()<<"\n";
    
    std::cout<<"dq:\n";
    std::cout<<getJointVelocities().transpose()<<"\n";

    std::cout<<"Hroot:\n";
    std::cout<<getFreeFlyerPosition()<<"\n";
    
    std::cout<<"Troot:\n";
//    std::cout<<getFreeFlyerVelocity().transpose()<<"\n";

    std::cout<<"total_mass:\n";
    std::cout<<getMass()<<"\n";

    std::cout<<"M:\n";
    std::cout<<getInertiaMatrix()<<"\n";
    
    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";

    std::cout<<"B:\n";
    std::cout<<getDampingMatrix()<<"\n";
    
    std::cout<<"n:\n";
    std::cout<<getNonLinearTerms()<<"\n";
    
    std::cout<<"g:\n";
    std::cout<<getGravityTerms()<<"\n";

    std::cout<<"l:\n";
    std::cout<<getLinearTerms()<<"\n";

    std::cout<<"comPosition:\n";
    std::cout<<getCoMPosition().transpose()<<"\n";

    std::cout<<"comVelocity:\n";
    std::cout<<getCoMVelocity().transpose()<<"\n";

    std::cout<<"comJdotQdot:\n";
    std::cout<<getCoMJdotQdot().transpose()<<"\n";

    std::cout<<"comJacobian:\n";
    std::cout<<getCoMJacobian()<<"\n";

    std::cout<<"comJacobianDot:\n";
    std::cout<<getCoMJacobianDot()<<"\n";

    
    
    for (int idx=0; idx<nbSegments(); idx++)
    {
        std::cout<<"segPosition "<<idx<<":\n";
        std::cout<<getSegmentPosition(idx)<<"\n";
    
        std::cout<<"segVelocity "<<idx<<":\n";
        std::cout<<getSegmentVelocity(idx)<<"\n";
    
        std::cout<<"segMass "<<idx<<":\n";
        std::cout<<getSegmentMass(idx)<<"\n";
    
        std::cout<<"segCoM "<<idx<<":\n";
        std::cout<<getSegmentCoM(idx)<<"\n";
    
        std::cout<<"segMassMatrix "<<idx<<":\n";
        std::cout<<getSegmentMassMatrix(idx)<<"\n";
    
        std::cout<<"segMomentsOfInertia "<<idx<<":\n";
        std::cout<<getSegmentMomentsOfInertia(idx)<<"\n";
    
        std::cout<<"segInertiaAxes "<<idx<<":\n";
        std::cout<<getSegmentInertiaAxes(idx)<<"\n";
    
        std::cout<<"segJacobian "<<idx<<":\n";
        std::cout<<getSegmentJacobian(idx)<<"\n";
    
        std::cout<<"segJdot "<<idx<<":\n";
        std::cout<<getSegmentJdot(idx)<<"\n";
    
        std::cout<<"segJointJacobian "<<idx<<":\n";
        std::cout<<getJointJacobian(idx)<<"\n";
    
        std::cout<<"segJdotQdot "<<idx<<":\n";
        std::cout<<getSegmentJdotQdot(idx).transpose()<<"\n";
    
    }

}
