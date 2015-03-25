#ifndef __ISIR_CONTROLLER_THREAD_XDE__H__
#define __ISIR_CONTROLLER_THREAD_XDE__H__
#include <Python.h>
#include <dictobject.h>
#include <Eigen/Core>
#include <Eigen/Lgsm>


#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

#include <xdecore/gvm.h>
#include <xdecore/gvm/DynamicModel.h>
#include "orcisir/ISIRController.h"
#include "orcisir/Solvers/OneLevelSolver.h"
//#include "orcisir/Models/XdeModel.h"
#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"
#include "orcXdeModel.h"

//XDEDynamicModel* getModelFromXDEDynamicModel(xde::gvm::extra::DynamicModel& xdeModel)
//{
//    return new XDEDynamicModel(xdeModel);
//}

class ISIRControllerThreadXDE: public RTT::TaskContext{

	public:
		ISIRControllerThreadXDE(const std::string& name);
		~ISIRControllerThreadXDE();

		bool startHook();
		void stopHook();
		bool configureHook();
		void updateHook();

        void setTimeStep(RTT::Seconds dt);
		void setDynModelPointerStr(const std::string& dynPtrStr, const std::string& rname, const std::string& jmapPtrStr, const std::string& sname);
		void loadAgent(std::string name);

	private:

		bool useReducedProblem;

		xde::gvm::extra::DynamicModel* robot;
        std::string robotName;
        std::string scenarioName;
		PyDictObject* jointMap;
		orcisir::OneLevelSolverWithQLD* internalSolver;
		orcXdeModel* orcModel;
   	    orcisir::ISIRController* ISIRctrl;
        orcisir::ISIRTaskManagerCollectionBase* taskScenario;

		void setISIRController();

        // Input ports
		RTT::InputPort< Eigen::VectorXd > port_in_q;
		RTT::InputPort< Eigen::VectorXd > port_in_qdot;
		RTT::InputPort< Eigen::Displacementd > port_in_d;
		RTT::InputPort< Eigen::Twistd > port_in_t;
		//RTT::InputPort< Eigen::VectorXd > in_contacts;

        // States to store the input ports values
		Eigen::VectorXd q;
		Eigen::VectorXd qdot;
		Eigen::Displacementd d;
		Eigen::Twistd t;
		//Eigen::VectorXd contacts;

        // Output port
		RTT::OutputPort< Eigen::VectorXd > port_out_tau;
    
        RTT::Seconds dt_sim;
        RTT::Seconds time_sim;

};

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( ISIRControllerThreadXDE );

#endif
