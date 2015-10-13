/**
 * \file ContactState.cpp
 * \author Darwin Lau
 *
 * \brief Implementation of the ContactState
 */

#include "dynamic_balance_3D/contactProperties.h"
#include "dynamic_balance_3D/contactState.h"

ContactState::ContactState(const ContactProperties& p)
    : prop(p), _disp(Eigen::Displacementd(Eigen::Vector3d::Zero(), Eigen::Rotation3d::Identity())), _inContact(0)
{
}

void ContactState::setState(Eigen::Displacementd d, int inContact)
{
    _disp = d;
    _inContact = inContact;
}
