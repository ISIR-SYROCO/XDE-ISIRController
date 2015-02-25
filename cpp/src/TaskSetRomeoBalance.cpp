#include "TaskSetRomeoBalance.h"

#ifndef PI
#define PI 3.1415926
#endif

TaskSetRomeoBalance::TaskSetRomeoBalance() : orcisir::ISIRTaskManagerCollectionBase()
{
}

TaskSetRomeoBalance::~TaskSetRomeoBalance()
{
}

void TaskSetRomeoBalance::doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model)
{
    // Initialise full posture task
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
    q_full[model.getDofIndex("LShoulderPitch")] = PI/2.0;
    q_full[model.getDofIndex("RShoulderPitch")] = PI/2.0;
    q_full[model.getDofIndex("LElbowRoll")] = -PI/2.0;
    q_full[model.getDofIndex("RElbowRoll")] = PI/2.0;   

    taskManagers["tmFull"] = new orcisir::ISIRFullPostureTaskManager(ctrl, model, "fullPostureTask", orc::FullState::INTERNAL, 9.0, 6.0, 0.000001, q_full);
    tmFull = dynamic_cast<orcisir::ISIRFullPostureTaskManager*>(taskManagers["tmFull"]);

    // Initialise partial posture task
    Eigen::VectorXi sdofs(1);
    sdofs << model.getDofIndex("TrunkYaw");
    Eigen::VectorXd zero = PI/4.0*Eigen::VectorXd::Ones(1);

    taskManagers["tmPartialTorso"] = new orcisir::ISIRPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", orc::FullState::INTERNAL, sdofs, 9.0, 6.0, 0.00001, zero);
    tmPartialTorso = dynamic_cast<orcisir::ISIRPartialPostureTaskManager*>(taskManagers["tmPartialTorso"]);

    // Initialise left foot contacts
    double mu_sys = 0.5;
    double margin = 0.0;
    Eigen::Displacementd LFContacts[4];
    LFContacts[0] = Eigen::Displacementd(0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    LFContacts[1] = Eigen::Displacementd(-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    LFContacts[2] = Eigen::Displacementd(0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    LFContacts[3] = Eigen::Displacementd(-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0);

    taskManagers["tmFootContactLeft"] = new orcisir::ISIRContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_ankle", LFContacts, 4, mu_sys, margin);
    tmFootContactLeft = dynamic_cast<orcisir::ISIRContactSetTaskManager*>(taskManagers["tmFootContactLeft"]);

    // Initailise right foot contacts
    Eigen::Displacementd RFContacts[4];
    RFContacts[0] = Eigen::Displacementd(0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    RFContacts[1] = Eigen::Displacementd(-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    RFContacts[2] = Eigen::Displacementd(0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0);
    RFContacts[3] = Eigen::Displacementd(-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0);

    taskManagers["tmFootContactRight"] = new orcisir::ISIRContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_ankle", RFContacts, 4, mu_sys, margin);
    tmFootContactRight = dynamic_cast<orcisir::ISIRContactSetTaskManager*>(taskManagers["tmFootContactRight"]);

    // Initialise CoM task
    taskManagers["tmCoM"] = new orcisir::ISIRCoMTaskManager(ctrl, model, "comTask", 9.0, 6.0, 0.5, Eigen::Vector3d(0.0,0.095,-0.259));
    tmCoM = dynamic_cast<orcisir::ISIRCoMTaskManager*>(taskManagers["tmCoM"]);

    // Initialise head task
    taskManagers["tmSegCartHead"] = new orcisir::ISIRSegCartesianTaskManager(ctrl, model, "headCartesianTask", "HeadPitchLink", orc::XYZ, 3.0, 6.0, 0.001, Eigen::Vector3d(0.0,0.095,-0.259+0.62));
    tmSegCartHead = dynamic_cast<orcisir::ISIRSegCartesianTaskManager*>(taskManagers["tmSegCartHead"]);

    // Initialise left foot task
    taskManagers["tmSegPoseFootLeft"] = new orcisir::ISIRSegPoseTaskManager(ctrl, model, "leftFootPoseTask", "l_ankle", orc::XYZ, 9.0, 6.0, 0.1, Eigen::Displacementd(0.0,0.095,-0.80,1.0,0.0,0.0,0.0));
    tmSegPoseFootLeft = dynamic_cast<orcisir::ISIRSegPoseTaskManager*>(taskManagers["tmSegPoseFootLeft"]);

    // Initialise right foot task
    taskManagers["tmSegPoseFootRight"] = new orcisir::ISIRSegPoseTaskManager(ctrl, model, "rightFootPoseTask", "r_ankle", orc::XYZ, 9.0, 6.0, 10.0, Eigen::Displacementd(0.05,-0.085,-0.65,1.0,0.0,0.0,0.0));
    tmSegPoseFootRight = dynamic_cast<orcisir::ISIRSegPoseTaskManager*>(taskManagers["tmSegPoseFootRight"]);
}

void TaskSetRomeoBalance::doUpdate(double time, orcisir::ISIRModel& state, void** args)
{
    static int act = 0;
    if (act == 0 && time > 2.0)
    {
        act = 1;
        tmFootContactRight->deactivate();
    }
    std::cout << "Time: " << time << ", CoM task error: " << tmCoM->getTaskErrorNorm() << std::endl;
    std::cout << "Time: " << time << ", foot right task error: " << tmSegPoseFootRight->getTaskErrorNorm() << std::endl;
}

