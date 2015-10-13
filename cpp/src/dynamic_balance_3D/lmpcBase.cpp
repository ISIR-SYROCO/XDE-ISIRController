/**
 * \file LMPC_Base.h
 * \author Darwin Lau
 *
 * \brief Base MPC class for LTI (Linear Time Invariant) Systems
 *  The aim of the base class is just to construct S and T matrices within X_k = S U_k + T x_k 
 */

#include "dynamic_balance_3D/lmpcBase.h"

LMPC_Base::LMPC_Base(const std::vector<Eigen::MatrixXd>& A_i, const std::vector<Eigen::MatrixXd>& B_i) 
    : dimState_i(B_i[0].rows()), dimInput_i(B_i[0].cols()), N_h(A_i.size())
{
    Av = A_i;
    Bv = B_i;
    //N_h = Av.size();

    dimState = N_h*dimState_i;
    dimInput = N_h*dimInput_i;

    S = Eigen::MatrixXd::Zero(dimState, dimInput);
    T = Eigen::MatrixXd::Zero(dimState, dimState_i);

    // Initialise S and T with the initial elements
    for (int i = 0; i < N_h; i++)
    {
        T.block(dimState_i*i, 0, dimState_i, dimState_i) = Av[0];

        for (int j = 0; j <= i; j++)
        {
            S.block(dimState_i*i, dimInput_i*j, dimState_i, dimInput_i) = Bv[j];
        }
    }

    // Actually compute S and T
    for (int i = 1; i < N_h; i++)
    {
        T.block(dimState_i*i, 0, dimState_i, dimState_i) = Av[i] * T.block(dimState_i*(i-1), 0, dimState_i, dimState_i);
    }

    for (int i = 0; i < N_h; i++)
    {
        for (int j = i+1; j < N_h; j++)
        {
            S.block(dimState_i*j, dimInput_i*i, dimState_i, dimInput_i) = Av[j] * S.block(dimState_i*(j-1), dimInput_i*i, dimState_i, dimInput_i);
        }
    }
    
    S_t = S.transpose();
    T_t = T.transpose();
}
