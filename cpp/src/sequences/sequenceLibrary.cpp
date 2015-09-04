#include "sequences/sequenceLibrary.h"

// Romeo sequences
#include "sequences/romeo/sequencesRomeo.h"
// iCub sequences
#include "sequences/icub/examples_basic/01_standing.h"
#include "sequences/icub/examples_basic/02_squatting.h"

wocra::wOcraTaskSequenceBase* SequenceLibrary::LoadSequence(const std::string& name)
{
    if (name == "sequence_iCub_01_standing")
        return new Sequence_iCub_01_Standing();
    if (name == "sequence_iCub_02_squatting")
        return new Sequence_iCub_02_Squatting();
    else if (name == "sequence_Romeo_balance")
        return new SequenceRomeo_Balance();
    else
        throw std::runtime_error(std::string("[LoadScenario()]: Error - Scenario name cannot be found."));
}
