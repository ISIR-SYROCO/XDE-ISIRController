#ifndef SEQUENCE_ICUB_02_SQUATTING_H
#define SEQUENCE_ICUB_02_SQUATTING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"

class Sequence_iCub_02_Squatting: public wocra::wOcraTaskSequenceBase
{
    public:
        Sequence_iCub_02_Squatting();
        virtual ~Sequence_iCub_02_Squatting();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
    private:
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
};

#endif 
