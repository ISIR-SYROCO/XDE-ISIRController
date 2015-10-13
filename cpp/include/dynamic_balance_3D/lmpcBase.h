/**
 * \file LMPC_Base.h
 * \author Darwin Lau
 *
 * \brief Base MPC class for LTI (Linear Time Invariant) Systems
 *  The aim of the base class is just to construct S and T matrices within X_k = S U_k + T x_k 
 */
#include <Eigen/Dense>
#include <vector>

#ifndef LMPCBASE_H
#define LMPCBASE_H

class LMPC_Base 
{
    public:
        LMPC_Base(const std::vector<Eigen::MatrixXd>& A_i, const std::vector<Eigen::MatrixXd>& B_i);
        
        int N_h;
        
    protected:
        std::vector<Eigen::MatrixXd> Av;
        std::vector<Eigen::MatrixXd> Bv;

        Eigen::MatrixXd S;
        Eigen::MatrixXd S_t;
        Eigen::MatrixXd T;
        Eigen::MatrixXd T_t;

        Eigen::VectorXd X;
        Eigen::VectorXd U;
        Eigen::VectorXd x0;

        // Dimensions for u_i and x_i (at one time instance)
        int dimInput_i;
        int dimState_i;
        // Dimensions for U_k and X_k (for the entire horizon)
        int dimInput;
        int dimState;
};

#endif
