#include "scenarios/ScenarioLibrary.h"

#include "scenarios/romeo/ScenariosRomeo.h"
#include "scenarios/icub/ScenariosICub.h"

wocra::wOcraTaskSequenceBase* ScenarioLibrary::LoadScenario(const std::string& name)
{
    if (name == "ScenarioICub_01_Standing")
        return new ScenarioICub_01_Standing();
    else if (name == "ScenarioRomeo_Balance")
        return new ScenarioRomeo_Balance();
    else
        throw std::runtime_error(std::string("[LoadScenario()]: Error - Scenario name cannot be found."));
}
