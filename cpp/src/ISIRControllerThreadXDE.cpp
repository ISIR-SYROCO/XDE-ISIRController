#include "ISIRControllerThreadXDE.h"
//#include "orcisir/Models/XdeModel.h"
#include <rtt/FlowStatus.hpp>
#include <rtt/TaskContext.hpp>
#include <orocos/ocl/DeploymentComponent.hpp>
#include <sstream>
#include <ctime>
#include "orcXdeModel.h"

#include "sequences/sequenceLibrary.h"

ISIRControllerThreadXDE::ISIRControllerThreadXDE(const std::string& name)
    : TaskContext(name)
    , robot(NULL)
{
    time_sim = 0.0;
    // Connect ports
    this->addPort("q", port_in_q);
    this->addPort("qdot", port_in_qdot);
    this->addPort("d", port_in_d);
    this->addPort("t", port_in_t);
//    this->addPort("contacts", in_contacts);

    this->addPort("tau", port_out_tau);

	this->addOperation("setTimeStep", &ISIRControllerThreadXDE::setTimeStep, this, RTT::OwnThread);
	this->addOperation("setDynModel", &ISIRControllerThreadXDE::setDynModelPointerStr, this, RTT::OwnThread);
	this->addOperation("loadPhy", &ISIRControllerThreadXDE::loadAgent, this, RTT::OwnThread);
}

ISIRControllerThreadXDE::~ISIRControllerThreadXDE()
{
    if (ISIRctrl != NULL) delete ISIRctrl;
    if (orcModel != NULL) delete orcModel;
    if (robot != NULL) delete robot;
    if (taskSequence != NULL) delete taskSequence;
}

void ISIRControllerThreadXDE::loadAgent(std::string name){
	OCL::DeploymentComponent deploy;
	bool loaded = deploy.import(name);
	if(loaded == true){
		std::cout << "loaded" << loaded << std::endl;
	}
}

bool ISIRControllerThreadXDE::startHook(){
    //gettimeofday(&initSysTime, NULL);
    return true;
}

void ISIRControllerThreadXDE::stopHook(){
    // Set all torques to zero when hook is stopped
    Eigen::VectorXd output = Eigen::VectorXd::Zero(robot->nbDofs());
    port_out_tau.write(output);
}

bool ISIRControllerThreadXDE::configureHook(){
    return true;
}

void ISIRControllerThreadXDE::updateHook(){
    struct timeval startTime;
    struct timeval endTime;
    gettimeofday(&startTime, NULL);

	RTT::FlowStatus flowStatus;
	
	flowStatus = port_in_q.read(q);
	if (flowStatus == RTT::NewData)
	{
	    robot->setJointPositions(q);
		orcModel->setJointPositions(q);
	}

	flowStatus = port_in_qdot.read(qdot);
	if (flowStatus == RTT::NewData)
	{
	    robot->setJointVelocities(qdot);
		orcModel->setJointVelocities(qdot);        
	}

	flowStatus = port_in_d.read(d);
	if (flowStatus == RTT::NewData)
	{
		robot->setFreeFlyerPosition(d);
		orcModel->setFreeFlyerPosition(d);
	}

	flowStatus = port_in_t.read(t);
	if (flowStatus == RTT::NewData)
	{
		robot->setFreeFlyerVelocity(t);
		orcModel->setFreeFlyerVelocity(t);
	}

/*	
	flowStatus = in_contacts.read(contacts);
	if (flowStatus == RTT::NewData) 
	{	
		std::cout << contacts.size() << std::endl;
	}
*/

    if (taskSequence == NULL || ISIRctrl == NULL)
    {
        throw std::runtime_error(std::string("[ISIRControllerThreadXDE::updateHook()]: Error - taskSequence or ISIRctrl has not been set"));        
    } 

    taskSequence->update(time_sim, *orcModel, NULL);
	Eigen::VectorXd tau(robot->nbDofs());
	ISIRctrl->computeOutput(tau);
    port_out_tau.write(tau);

    gettimeofday(&endTime, NULL);
    //time in milliseconds:
    double tval =  1000 * ( endTime.tv_sec - startTime.tv_sec ) + ( endTime.tv_usec - startTime.tv_usec ) / 1000;

//    printf("[PERFORMANCE INFORMATION]:\n");
//    printf("Expected period %3.3f ms. Real duration: %3.3f ms.\n", tval, dt_sim*1000);
    time_sim += dt_sim;
}


/////////////////////////////////////////////////////////////////////////////////////////
// OPERATIONS
/////////////////////////////////////////////////////////////////////////////////////////

void ISIRControllerThreadXDE::setISIRController()
{
    useReducedProblem = false;
    internalSolver = new wocra::OneLevelSolverWithQLD();
    orcModel = new orcXdeModel(robot, robotName, jointMap);
    ISIRctrl = new wocra::wOcraController("myCtrl", *orcModel, *internalSolver, useReducedProblem);
}


void ISIRControllerThreadXDE::setDynModelPointerStr(const std::string& dynModelPtrStr, const std::string& rname, const std::string& jmapPtrStr, const std::string& sname)
{
    long long dynModelPtr = atoll(dynModelPtrStr.c_str());
    long long jmapPtr = atoll(jmapPtrStr.c_str());
    robot = reinterpret_cast<xde::gvm::extra::DynamicModel*>(dynModelPtr);
    jointMap = reinterpret_cast<PyDictObject*>(jmapPtr);
    robotName = rname;
    scenarioName = sname;

    setISIRController();

    taskSequence = SequenceLibrary::LoadSequence(sname);
    taskSequence->init(*ISIRctrl, *orcModel);
}

void ISIRControllerThreadXDE::setTimeStep(RTT::Seconds _dt)
{
    dt_sim = _dt;
    setPeriod(dt_sim);
}
