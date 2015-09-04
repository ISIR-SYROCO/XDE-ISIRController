#ifndef SEQUENCE_ICUB_01_STANDING_H
#define SEQUENCE_ICUB_01_STANDING_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"

class Sequence_iCub_01_Standing: public wocra::wOcraTaskSequenceBase
{
    public:
        Sequence_iCub_01_Standing();
        virtual ~Sequence_iCub_01_Standing();
    protected: 
        virtual void doInit(wocra::wOcraController& ctrl, wocra::wOcraModel& model);
        virtual void doUpdate(double time, wocra::wOcraModel& state, void** args); 
/*
    private:
        // Full posture task
        wocra::wOcraFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        wocra::wOcraPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        wocra::wOcraContactSetTaskManager*             tmFootContactRight;
        // Waist task
        wocra::wOcraSegPoseTaskManager*                tmSegPoseWaist;
*/
};

#endif 
