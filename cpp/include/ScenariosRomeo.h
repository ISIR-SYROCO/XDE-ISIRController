#ifndef SCENARIOSROMEO_H
#define SCENARIOSROMEO_H

#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"
#include "orcisir/Models/ISIRModel.h"

class ScenarioRomeo_Balance: public orcisir::ISIRTaskManagerCollectionBase
{
    public:
        ScenarioRomeo_Balance();
        virtual ~ScenarioRomeo_Balance();
    protected: 
        virtual void doInit(orcisir::ISIRController& ctrl, orcisir::ISIRModel& model);
        virtual void doUpdate(double time, orcisir::ISIRModel& state, void** args); 
    private:
        // Full posture task
        orcisir::ISIRFullPostureTaskManager*            tmFull;
        // Partial torso posture task
        orcisir::ISIRPartialPostureTaskManager*         tmPartialTorso;
        // Left foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactLeft;
        // Right foot contact task
        orcisir::ISIRContactSetTaskManager*             tmFootContactRight;
        // CoM task
        orcisir::ISIRCoMTaskManager*                    tmCoM;
        // Segment head task
        orcisir::ISIRSegCartesianTaskManager*           tmSegCartHead;
        // Segment left foot task
        orcisir::ISIRSegPoseTaskManager*                tmSegPoseFootLeft;
        // Segment right foot task
        orcisir::ISIRSegPoseTaskManager*                tmSegPoseFootRight;
};

#endif // TASKSETROMEOBALANCE_H
