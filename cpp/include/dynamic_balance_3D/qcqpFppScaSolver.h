/**
 * \file QCQP_FPPSCA_Solver.h
 * \author Darwin Lau
 *
 * \brief Solving QCQP through the FPPSCA method
 */
#include <Eigen/Dense>
#include <vector>

#include "gurobi_c++.h"

#ifndef QCQP_FPPSCA_SOLVER_H
#define QCQP_FPPSCA_SOLVER_H

class QCQP_FPPSCA_Solver 
{
    public:
        QCQP_FPPSCA_Solver(double weight_fpp, int max_sca, double convergence_tol);
        ~QCQP_FPPSCA_Solver();
        // Solves the following problem
        // X_opt =  argmin_X            X' * H * X + f' * X
        //          subject to :        A_eq * X = b_eq
        //                              A_ineq * X <= b_eq
        //                              X' * Pc_i * X + qc_i' * X <= rc_i
        void solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq, const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq, const std::vector<Eigen::MatrixXd>& Pc, const std::vector<Eigen::VectorXd>& qc, const std::vector<double>& rc, const Eigen::VectorXd& x0);

        // Optimal X value
        Eigen::VectorXd X_opt;
        // Optimal slack S value
        Eigen::VectorXd S_opt;
        // Optimal value
        double OptVal;

    private:
        double weight_fpp;
        int max_sca;
        double convergence_tol;
    
        GRBEnv gurobi_env;
};

#endif
