#include "scenarios/romeo/ScenariosRomeo.h"

#ifndef PI
#define PI 3.1415926
#endif

ScenarioRomeo_Balance::ScenarioRomeo_Balance() : wocra::wOcraTaskSequenceBase()
{
}

ScenarioRomeo_Balance::~ScenarioRomeo_Balance()
{
}

void ScenarioRomeo_Balance::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    // Initialise full posture task
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
    q_full[model.getDofIndex("LShoulderPitch")] = PI/2.0;
    q_full[model.getDofIndex("RShoulderPitch")] = PI/2.0;
    q_full[model.getDofIndex("LElbowRoll")] = -PI/2.0;
    q_full[model.getDofIndex("RElbowRoll")] = PI/2.0;   

    taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 9.0, 6.0, 0.000001, q_full);
    tmFull = dynamic_cast<wocra::wOcraFullPostureTaskManager*>(taskManagers["tmFull"]);

    // Initialise partial posture task
    Eigen::VectorXi sdofs(1);
    sdofs << model.getDofIndex("TrunkYaw");
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);

    taskManagers["tmPartialTorso"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureTorsoTask", ocra::FullState::INTERNAL, sdofs, 9.0, 6.0, 0.00001, zero);
    tmPartialTorso = dynamic_cast<wocra::wOcraPartialPostureTaskManager*>(taskManagers["tmPartialTorso"]);

    // Initialise left foot contacts
    double mu_sys = 0.5;
    double margin = 0.0;
    std::vector<Eigen::Displacementd> LFContacts;
    LFContacts.push_back(Eigen::Displacementd(0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    LFContacts.push_back(Eigen::Displacementd(-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    LFContacts.push_back(Eigen::Displacementd(0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    LFContacts.push_back(Eigen::Displacementd(-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0));

    taskManagers["tmFootContactLeft"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_ankle", LFContacts, mu_sys, margin);
    tmFootContactLeft = dynamic_cast<wocra::wOcraContactSetTaskManager*>(taskManagers["tmFootContactLeft"]);

    // Initailise right foot contacts
    std::vector<Eigen::Displacementd> RFContacts;
    RFContacts.push_back(Eigen::Displacementd(0.18, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    RFContacts.push_back(Eigen::Displacementd(-0.06, -0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    RFContacts.push_back(Eigen::Displacementd(0.18, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0));
    RFContacts.push_back(Eigen::Displacementd(-0.06, 0.03, -0.06, 0.0, 1.0, 0.0, 0.0));

    taskManagers["tmFootContactRight"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_ankle", RFContacts, mu_sys, margin);
    tmFootContactRight = dynamic_cast<wocra::wOcraContactSetTaskManager*>(taskManagers["tmFootContactRight"]);

    // Initialise CoM task
    taskManagers["tmCoM"] = new wocra::wOcraCoMTaskManager(ctrl, model, "comTask", ocra::XYZ, 9.0, 6.0, 0.5, Eigen::Vector3d(0.0,0.0,-0.220));
    tmCoM = dynamic_cast<wocra::wOcraCoMTaskManager*>(taskManagers["tmCoM"]);

    // Initialise body task
    taskManagers["tmSegOrientationBody"] = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "bodyOrientationTask", "body", 3.0, 6.0, 0.001, Eigen::Rotation3d::Identity());
    tmSegOrientationBody = dynamic_cast<wocra::wOcraSegOrientationTaskManager*>(taskManagers["tmSegOrientationBody"]);

    // Initialise left foot task
    taskManagers["tmSegPoseFootLeft"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "leftFootPoseTask", "l_ankle", ocra::XYZ, 9.0, 6.0, 0.1, Eigen::Displacementd(0.0,0.095,-0.80,1.0,0.0,0.0,0.0));
    tmSegPoseFootLeft = dynamic_cast<wocra::wOcraSegPoseTaskManager*>(taskManagers["tmSegPoseFootLeft"]);

    // Initialise right foot task
    taskManagers["tmSegPoseFootRight"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "rightFootPoseTask", "r_ankle", ocra::XYZ, 9.0, 6.0, 10.0, Eigen::Displacementd(0.05,-0.085,-0.65,1.0,0.0,0.0,0.0));
    tmSegPoseFootRight = dynamic_cast<wocra::wOcraSegPoseTaskManager*>(taskManagers["tmSegPoseFootRight"]);
}

void ScenarioRomeo_Balance::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
    static int actFoot = 0;
    static int actCoM = 0;
    if (actCoM == 0 && time > 5.0)
    {
        actCoM = 1;
        tmCoM->setState(Eigen::Vector3d(0.0,0.095,-0.256));
    }
    if (actFoot == 0 && time > 10.0)
    {
        actFoot = 1;
        tmFootContactRight->deactivate();
    }
    std::cout << "Time: " << time << ", CoM task error: " << tmCoM->getTaskErrorNorm() << std::endl;
    std::cout << "Time: " << time << ", foot right task error: " << tmSegPoseFootRight->getTaskErrorNorm() << std::endl;
}

