#ifndef SCENARIO_LIBRARY_H
#define SCENARIO_LIBRARY_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"

class ScenarioLibrary {
    public:
        static wocra::wOcraTaskSequenceBase* LoadScenario(const std::string& name);
};

#endif
