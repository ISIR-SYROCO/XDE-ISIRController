/**
 * \file MPCTrajectoryGenerator.h
 * \author Darwin Lau
 *
 * \brief Used to generate the trajectory for the MPC
 */
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include "dynamic_balance_3D/mpcModelState.h"
#include "dynamic_balance_3D/contactProperties.h"

#ifndef MPCTRAJECTORYGENERATOR_H
#define MPCTRAJECTORYGENERATOR_H

#define LF_CONTACT_IND 0
#define RF_CONTACT_IND 1
#define LH_CONTACT_IND 2
#define RH_CONTACT_IND 3

class MPCTrajectoryGenerator 
{
    public:
        static void generateStraightWalkY(double dt, int N, int numSteps, double strideLength, double strideWidth, double strideTime, double doubleStanceTime, double preTime, double postTime, const std::vector<ContactProperties> cProp, int lMoving, double zHeight, double z0, std::vector<MPCModelState>& trajectory, Eigen::VectorXd& time, double& timeTotal, MPCModelState& x0);

};

#endif
