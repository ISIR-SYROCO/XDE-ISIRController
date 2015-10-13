#include "sequences/icub/examples_basic/03_dbal_standing.h"
#include "dynamic_balance_3D/balanceMPC.h"
#include <cmath>

#ifndef PI
#define PI 3.1415926
#endif

#define W_COM 1
#define DT 0.1
#define W_JERK 0.02 * DT * DT * DT
#define NUM_CONTACTS 2
#define LF_CONTACT_IND 0
#define RF_CONTACT_IND 1

Sequence_iCub_03_Dbal_Standing::Sequence_iCub_03_Dbal_Standing() : wocra::wOcraTaskSequenceBase(), balance_mpc(Eigen::VectorXd::Zero(1), 0.0, W_COM, W_JERK)
{
}

Sequence_iCub_03_Dbal_Standing::~Sequence_iCub_03_Dbal_Standing()
{
}

void Sequence_iCub_03_Dbal_Standing::doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model)
{
    // Setup the MPC for balancing
    Eigen::VectorXd mpc_dtVec = Eigen::VectorXd::Ones(7);
    //mpc_dtVec << 0.01, 0.01, 0.2, 0.2, 0.2;
    mpc_dtVec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    mpc_Nh = mpc_dtVec.size();
    balance_mpc = BalanceMPC(mpc_dtVec, model.getMass(), W_COM, W_JERK);
    mpc_Uk = Eigen::VectorXd::Zero(N_DIMS * mpc_Nh);
    mpc_contactProps.resize(NUM_CONTACTS, ContactProperties(0.099, -0.031, -0.027, 0.027));
    mpc_trajref_k.resize(mpc_Nh, mpc_contactProps);
    
    mpc_xk = MPCModelState(mpc_contactProps);
    mpc_xk.x_G = Eigen::Vector3d::Zero();
    mpc_xk.x_G_d = Eigen::Vector3d::Zero();
    mpc_xk.x_G_dd = Eigen::Vector3d::Zero();

    Eigen::Vector3d initCoM = Eigen::Vector3d(0.0, 0.0, 0.5);
    mpc_xk.x_G = initCoM;

    for (int i = 0; i < mpc_Nh; i++)
    {
        mpc_trajref_k[i].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(Eigen::Vector3d(-0.05, 0.0, 0.0), Eigen::Rotation3d::Identity()), IN_CONTACT);
        mpc_trajref_k[i].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(Eigen::Vector3d(0.05, 0.0, 0.0), Eigen::Rotation3d::Identity()), IN_CONTACT);
        mpc_trajref_k[i].x_G = Eigen::Vector3d(0.05, 0.0, 0.5);
    }

    // Initialise full posture task
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model.nbInternalDofs());
    q_full[model.getDofIndex("l_elbow_pitch")] = PI/8.0;
    q_full[model.getDofIndex("r_elbow_pitch")] = PI/8.0;
    q_full[model.getDofIndex("l_knee")] = -0.05;
    q_full[model.getDofIndex("r_knee")] = -0.05;
    q_full[model.getDofIndex("l_ankle_pitch")] = -0.05;
    q_full[model.getDofIndex("r_ankle_pitch")] = -0.05;
    q_full[model.getDofIndex("l_shoulder_roll")] = PI/8.0;
    q_full[model.getDofIndex("r_shoulder_roll")] = PI/8.0;

    taskManagers["tmFull"] = new wocra::wOcraFullPostureTaskManager(ctrl, model, "fullPostureTask", ocra::FullState::INTERNAL, 9.0, 2*sqrt(9.0), 0.0001, q_full, false);

    // Initialise waist pose
    //taskManagers["tmSegPoseWaist"] = new wocra::wOcraSegPoseTaskManager(ctrl, model, "waistPoseTask", "waist", ocra::XYZ, 36.0, 2*sqrt(36.0), 1.0, Eigen::Displacementd(0.0,0.0,0.58,-M_SQRT1_2,0.0,0.0,M_SQRT1_2), false);
    taskManagers["tmSegOrientationWaist"] = new wocra::wOcraSegOrientationTaskManager(ctrl, model, "waistPoseTask", "waist", 36.0, 2*sqrt(36.0), 1, Eigen::Rotation3d(-M_SQRT1_2,0.0,0.0,M_SQRT1_2), false);

    // Initialise partial posture task
    Eigen::VectorXi sdofs(3);
    sdofs << model.getDofIndex("torso_pitch"), model.getDofIndex("torso_roll"), model.getDofIndex("torso_yaw");

    Eigen::VectorXd zero = Eigen::VectorXd::Zero(3);

    taskManagers["tmPartialBack"] = new wocra::wOcraPartialPostureTaskManager(ctrl, model, "partialPostureBackTask", ocra::FullState::INTERNAL, sdofs, 16.0, 2*sqrt(16.0), 0.001, zero, false);


    // Initialise CoM task
    taskManagers["tmCoM"] = new wocra::wOcraCoMTaskManager(ctrl, model, "comTask", ocra::XYZ, 9.0, 2*sqrt(9.0), 10.0, initCoM, false);
    tmCoM = dynamic_cast<wocra::wOcraCoMTaskManager*>(taskManagers["tmCoM"]);

    double mu_sys = 0.5;
    double margin = 0.0;

    double sqrt2on2 = sqrt(2.0)/2.0;
    Eigen::Rotation3d rotLZdown = Eigen::Rotation3d(-sqrt2on2, 0.0, -sqrt2on2, 0.0) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);
    Eigen::Rotation3d rotRZdown = Eigen::Rotation3d(0.0, sqrt2on2, 0.0, sqrt2on2) * Eigen::Rotation3d(0.0, 1.0, 0.0, 0.0);

    // Initialise left foot contacts
    std::vector<Eigen::Displacementd> LFContacts;
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039,-.027,-.031), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039, .027,-.031), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039, .027, .099), rotLZdown));
    LFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039,-.027, .099), rotLZdown));


    taskManagers["tmFootContactLeft"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "leftFootContactTask", "l_foot", LFContacts, mu_sys, margin, false);

    // Initailise right foot contacts
    std::vector<Eigen::Displacementd> RFContacts;
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039,-.027, .031), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039, .027, .031), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039, .027,-.099), rotRZdown));
    RFContacts.push_back(Eigen::Displacementd(Eigen::Vector3d(-.039,-.027,-.099), rotRZdown));

    taskManagers["tmFootContactRight"] = new wocra::wOcraContactSetTaskManager(ctrl, model, "RightFootContactTask", "r_foot", RFContacts, mu_sys, margin, false);
}

void Sequence_iCub_03_Dbal_Standing::doUpdate(double time, wocra::wOcraModel& state, void** args)
{
    // Generate the input for the MPC

    // Compute the CoM from MPC
    balance_mpc.compute(mpc_xk, mpc_trajref_k, mpc_Uk);
    mpc_xk = balance_mpc.soln;
    // Set the Uk command for the next MPC round
    mpc_Uk.setZero(N_DIMS * mpc_Nh);
    mpc_Uk.segment(0, N_DIMS * (mpc_Nh - 1)) = balance_mpc.U_opt.segment(N_DIMS, N_DIMS * (mpc_Nh - 1));
    // Execute the motion on the CoM task manager
    //tmCoM->setState(mpc_xk.x_G);
    //tmCoM->setState(mpc_xk.x_G, mpc_xk.x_G_d, mpc_xk.x_G_dd);

    std::cout << "Time: " << time << std::endl;
//    std::cout << "MPC CoM position: " << mpc_xk.x_G.transpose() << std::endl;
//    std::cout << "True CoM position: " << state.getCoMPosition().transpose() << std::endl;

    // Execute the motion on the waist task manager
}
