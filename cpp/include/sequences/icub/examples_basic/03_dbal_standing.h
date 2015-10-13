#ifndef SEQUENCE_ICUB_03_DBALSTANDING_H
#define SEQUENCE_ICUB_03_DBALSTANDING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"
#include "dynamic_balance_3D/balanceMPC.h"
#include "dynamic_balance_3D/mpcModelState.h"
#include "wocra/Tasks/Managers/wOcraCoMTaskManager.h"

class Sequence_iCub_03_Dbal_Standing: public wocra::wOcraTaskSequenceBase
{
    public:
        Sequence_iCub_03_Dbal_Standing();
        virtual ~Sequence_iCub_03_Dbal_Standing();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
    private:
        wocra::wOcraCoMTaskManager* tmCoM;

        BalanceMPC balance_mpc;
        int mpc_Nh;
        Eigen::VectorXd mpc_Uk;
        MPCModelState mpc_xk;
        std::vector<MPCModelState> mpc_trajref_k;
        std::vector<ContactProperties> mpc_contactProps;

};

#endif 
