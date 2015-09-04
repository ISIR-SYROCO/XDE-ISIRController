#ifndef SCENARIO_LIBRARY_H
#define SCENARIO_LIBRARY_H

#include "wocra/Tasks/wOcraTaskSequenceBase.h"

class SequenceLibrary {
    public:
        static wocra::wOcraTaskSequenceBase* LoadSequence(const std::string& name);
};

#endif
