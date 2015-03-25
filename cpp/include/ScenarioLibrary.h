#ifndef SCENARIO_LIBRARY_H
#define SCENARIO_LIBRARY_H

#include "orcisir/Tasks/ISIRTaskManagerCollectionBase.h"

orcisir::ISIRTaskManagerCollectionBase* LoadScenario(const std::string& name);

#endif
