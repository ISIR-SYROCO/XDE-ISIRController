/**
 * \file MPCTrajectoryGenerator.cpp
 * \author Darwin Lau
 *
 * \brief Implementation of the MPCTrajectoryGenerator.cpp
 */
#include "dynamic_balance_3D/mpcTrajectoryGenerator.h"
#include "dynamic_balance_3D/balanceMPC.h"

#include <iostream>

void MPCTrajectoryGenerator::generateStraightWalkY(double dt, int N, int numSteps, double strideLength, double strideWidth, double strideTime, double doubleStanceTime, double preTime, double postTime, const std::vector<ContactProperties> cProp, int lMoving, double zHeight, double z0, std::vector<MPCModelState>& trajectory, Eigen::VectorXd& time, double& timeTotal, MPCModelState& x0)
{
    double strideSpeed = strideLength/strideTime;
    timeTotal = preTime + strideTime/2 + numSteps*(doubleStanceTime + strideTime) + (doubleStanceTime + strideTime/2) + postTime;
    double timeTotalN = timeTotal + dt*N;
    // 2 more, one for start, one for end
    int numTimeSteps = timeTotal/dt + 1;
    int numTimeStepsN = numTimeSteps + N;

    time.setLinSpaced(numTimeSteps, 0.0, timeTotal);

    trajectory.resize(numTimeStepsN, cProp); 


    Eigen::Vector3d lfeet_pos(-strideWidth/2, 0.0, 0.0); 
    Eigen::Vector3d rfeet_pos(strideWidth/2, 0.0, 0.0); 
    Eigen::Vector3d pfeet_pos;

    int ind_s, ind_e;
    int moving_ind;
    // Pre-time phase
    ind_s = 0;
    ind_e = ind_s + (int)(preTime/dt);
    for (int t = ind_s; t <= ind_e; t++)
    {
        trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
        trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
    } 

    // First half step
    ind_s = ind_e + 1;
    ind_e = ind_s + (int)(0.5*strideTime/dt) - 1;
    for (int t = ind_s; t <= ind_e; t++)
    {
        if (lMoving)
        {
            lfeet_pos[Y_IND] += dt*strideSpeed;
            trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
            trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
        }
        else
        {
            rfeet_pos[Y_IND] += dt*strideSpeed;
            trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
            trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
        }
    }
    lMoving = !lMoving;

    // Full steps 
    for (int count = 0; count < numSteps; count++)
    {
        // Double stance phase
        ind_s = ind_e + 1;
        ind_e = ind_s + (int)(doubleStanceTime/dt) - 1;
        for (int t = ind_s; t <= ind_e; t++)
        {
            trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
            trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
        }
        // Swing phase
        ind_s = ind_e + 1;
        ind_e = ind_s + (int)(strideTime/dt) - 1;
        for (int t = ind_s; t <= ind_e; t++)
        {
            if (lMoving)
            {
                lfeet_pos[Y_IND] += dt*strideSpeed;
                trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
                trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
            }
            else
            {
                rfeet_pos[Y_IND] += dt*strideSpeed;
                trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
                trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
            }
        }
        lMoving = !lMoving;
    }

    // Double stance phase
    ind_s = ind_e + 1;
    ind_e = ind_s + (int)(doubleStanceTime/dt) - 1;
    for (int t = ind_s; t <= ind_e; t++)
    {
        trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
        trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
    }

    // Last half step
    ind_s = ind_e + 1;
    ind_e = ind_s + (int)(0.5*strideTime/dt) - 1;
    for (int t = ind_s; t <= ind_e; t++)
    {
        if (lMoving)
        {
            lfeet_pos[Y_IND] += dt*strideSpeed;
            trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
            trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
        }
        else
        {
            rfeet_pos[Y_IND] += dt*strideSpeed;
            trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
            trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), N_IN_CONTACT);
        }
    }
    lMoving = !lMoving;

    // Post time 
    ind_s = ind_e + 1;
    ind_e = ind_s + (int)(postTime/dt) + N - 1;
    for (int t = ind_s; t <= ind_e; t++)
    {
        trajectory[t].contacts[LF_CONTACT_IND].setState(Eigen::Displacementd(lfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
        trajectory[t].contacts[RF_CONTACT_IND].setState(Eigen::Displacementd(rfeet_pos, Eigen::Rotation3d::Identity()), IN_CONTACT);
    }

    // Generate the CoM
    for (int t = 0; t < numTimeStepsN; t++)
    {
        trajectory[t].x_G = (trajectory[t].contacts[LF_CONTACT_IND].centre() + trajectory[t].contacts[RF_CONTACT_IND].centre())/2;
        trajectory[t].x_G[Z_IND] += zHeight; 
    }

    // Set initial xG position
    x0.x_G = Eigen::Vector3d::Zero();
    x0.x_G_d = Eigen::Vector3d::Zero();
    x0.x_G_dd = Eigen::Vector3d::Zero();

    x0.x_G[Z_IND] = z0;
    x0.x_G[X_IND] = 0.05;
}
