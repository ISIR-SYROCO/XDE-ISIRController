/**
 * \file CoM_MPC.h
 * \author Darwin Lau
 *
 * \brief CoM planner using MPC for given contacts
 */
#include <Eigen/Dense>
#include "dynamic_balance_3D/lmpcBase.h"
#include "dynamic_balance_3D/mpcModelState.h"
#include "dynamic_balance_3D/qcqpFppScaSolver.h"

#ifndef COM_MPC_H
#define COM_MPC_H

#define N_DIMS 3 // Dimensions X Y Z
#define N_LEVELS 3 // position, velocity, acceleration
#define N_STATES N_DIMS*N_LEVELS // number of states: (position, velocity, acceleration) in X, Y, and Z

#define X_IND 0
#define Y_IND 1
#define Z_IND 2

#define IN_CONTACT 1
#define N_IN_CONTACT !IN_CONTACT

#define FFPSCA_MAXSCA 100
#define FPPSCA_W_FPP 100000
#define FFPSCA_TOL 0.0001

#define GRAV_CONST 9.81

#define ENABLE_TERMINAL_CONSTRAINTS 1

class BalanceMPC : LMPC_Base
{
    public:
        BalanceMPC(const Eigen::VectorXd& dt, double mass, double w_CoM, double w_jerk);
        void compute(const MPCModelState& state, const std::vector<MPCModelState>& traj_ref, const Eigen::VectorXd& U_prev);

        std::vector<MPCModelState> soln_full;
        MPCModelState soln;

        Eigen::VectorXd U_opt;
        Eigen::VectorXd u_opt;
        Eigen::VectorXd X_opt;
        Eigen::VectorXd x_opt;
        
    private:
        Eigen::MatrixXd _generateA(double dt);
        Eigen::MatrixXd _generateB(double dt);
        std::vector<Eigen::MatrixXd> _generateA_vec(const Eigen::VectorXd& dt);
        std::vector<Eigen::MatrixXd> _generateB_vec(const Eigen::VectorXd& dt);

        Eigen::VectorXd _stateToX(const MPCModelState& s);
        Eigen::Matrix3d _skewSymmetricMatrix(const Eigen::Vector3d& v);
        void _XToState(const Eigen::VectorXd& s, Eigen::Vector3d& x_G, Eigen::Vector3d& x_G_d, Eigen::Vector3d& x_G_dd);

        Eigen::VectorXd dtVec;
        //int N_h;
        double mass;

        // weights
        double w_CoM;
        double w_jerk;

        // Matrices for single instance in horizon
        Eigen::MatrixXd T_pos;
        Eigen::MatrixXd T_posz;
        Eigen::MatrixXd T_acc;

        // Matrices for the entire horizon
        Eigen::MatrixXd Q_CoM;
        Eigen::MatrixXd Q_jerk;
        Eigen::MatrixXd T_P;
        Eigen::MatrixXd T_Pz;
        Eigen::MatrixXd T_A;
        Eigen::MatrixXd T_N;

        // Precomputed stuff
        Eigen::MatrixXd T_acc_t;
        Eigen::MatrixXd T_P_T;
        Eigen::MatrixXd Q_CoM_T_P_S;
        Eigen::MatrixXd T_N_t;

        // Gamma
        Eigen::Vector3d Gamma_G;
        std::vector<Eigen::Vector3d> Gamma;

        // Natrices used for the MPC
        Eigen::MatrixXd H; 

        QCQP_FPPSCA_Solver qcqp_solver;
};

#endif
