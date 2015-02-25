#ifndef TASKSETICUB_H
#define TASKSETICUB_H

#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"

class Task_01_Standing: public orcisir::ISIRTaskManagerCollectionBase
{
    public:
        Task_01_Standing();
        virtual ~Task_01_Standing();
    protected: 
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args); 
/*
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialBack;
        // Left foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactRight;
        // Waist task
        orcisir::ISIRSegPoseTaskManager*                tmSegPoseWaist;
*/
};

#endif // TASKSETROMEOBALANCE_H
