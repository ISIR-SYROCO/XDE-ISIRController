/**
 * \file MPCModelState.cpp
 * \author Darwin Lau
 *
 * \brief Implementation of the MPCModel
 */
#include "dynamic_balance_3D/mpcModelState.h"
#include <vector>

MPCModelState::MPCModelState()
{
}

MPCModelState::MPCModelState(const std::vector<ContactProperties>& c_props)
    : numContacts(c_props.size()), contactProps(c_props)
{
    contacts.reserve(c_props.size()); 

    for (int i = 0; i < numContacts; i++)
    {
        contacts.push_back(ContactState(c_props[i]));
        //contacts[i] = ContactState(c_props[i]);
    }
}

std::vector<ContactState> MPCModelState::getActiveContacts() const
{
    std::vector<ContactState> ac;
    for (int i = 0; i < numContacts; i++)
    {
        if (contacts[i].inContact())
        {
            ac.push_back(contacts[i]);
        }
    }
    return ac;
}

MPCModelState::~MPCModelState()
{
/*
    for (int i = 0; i < contacts.size(); i++)
    {
        delete contacts[i];
    }
*/
    contacts.clear();
}
