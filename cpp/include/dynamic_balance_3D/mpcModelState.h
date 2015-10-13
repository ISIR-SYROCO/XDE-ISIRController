/**
 * \file MPCModelState.h
 * \author Darwin Lau
 *
 * \brief Stores the state of the single mass model for the MPC
 */
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include "dynamic_balance_3D/contactProperties.h"
#include "dynamic_balance_3D/contactState.h"

#ifndef MPCMODELSTATE_H
#define MPCMODELSTATE_H

class MPCModelState 
{
    public:
        MPCModelState();
        MPCModelState(const std::vector<ContactProperties>& contact_props);
        ~MPCModelState();

        int numContacts;

        std::vector<ContactProperties> contactProps;

        std::vector<ContactState> contacts;

        Eigen::Vector3d x_G;
        Eigen::Vector3d x_G_d;
        Eigen::Vector3d x_G_dd;

        Eigen::Vector3d x_G_ddd;

        std::vector<ContactState> getActiveContacts() const;
    private:
};

#endif
