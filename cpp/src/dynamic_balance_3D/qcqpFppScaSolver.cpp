/**
 * \file QCQP_FPPSCA_Solver.cpp
 * \author Darwin Lau
 *
 * \brief Implementation of QCQP solver through the FPPSCA method
 */ 
#include <stdexcept>

#include "dynamic_balance_3D/qcqpFppScaSolver.h"
#include "gurobi_c++.h"
#include <Eigen/Eigenvalues>

QCQP_FPPSCA_Solver::QCQP_FPPSCA_Solver(double weight_fpp, int max_sca, double convergence_tol)
    : weight_fpp(weight_fpp), max_sca(max_sca), convergence_tol(convergence_tol)
{
}

QCQP_FPPSCA_Solver::~QCQP_FPPSCA_Solver()
{
//    delete gurobi_env;
}

void QCQP_FPPSCA_Solver::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq, const Eigen::MatrixXd& A_ineq, const Eigen::VectorXd& b_ineq, const std::vector<Eigen::MatrixXd>& Pc, const std::vector<Eigen::VectorXd>& qc, const std::vector<double>& rc, const Eigen::VectorXd& x0)
{
    try
    {
        GRBEnv grb_env = GRBEnv();
        grb_env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model = GRBModel(grb_env);

        OptVal = GRB_INFINITY;

        int n = 0;
        int n_qc = 0;

        // Check the problem size : if no linear term at all
        if (f.size() == 0 && H.cols() == 0)
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: No objective function since H and f are both empty"));
        else if (f.size() > 0)
            n = f.size();
        else 
            n = H.cols();
        // Check the problem size : if there is a quadratic term and its dimensions don't match the linear objective 
        if (H.cols() > 0 && (H.cols() != n || H.rows() != n))
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: H matrix with incorrect dimensions"));

        if (f.size() > 0 && (f.size() != n))
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: f vector with incorrect dimensions"));

        // Check the problem size : if the number of columns for A_eq * X = b_eq do not match the size of X
        if (A_eq.cols() > 0 && (A_eq.cols() != n))
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: Equality constraint A_eq with incorrect number of columns"));

        if (A_eq.rows() > 0 && (A_eq.rows() != b_eq.size()))
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: Equality constraint A_eq and b_eq dimensions do not match"));

        if (A_ineq.rows() > 0 && (A_ineq.rows() != b_ineq.size()))
            throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: Equality constraint A_ineq and b_ineq dimensions do not match"));

        n_qc = Pc.size();

        for (int i=0; i<n_qc; i++)
        {
            if (Pc[i].cols() != n || Pc[i].rows() != n)
                throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: Quadratic inequality constraint Hessian term dimensions do not match"));
            if (qc[i].size() != n)
                throw std::runtime_error(std::string("[QCQP_FPPSCA_SOlver::Solve()]: Quadratic inequality constraint linear term dimensions do not match"));
        }
    
        // Setup bounds for variables
        double lb[n + n_qc];
        double ub[n + n_qc];
        for (int i=0; i < n; i++)
        {
            lb[i] = -GRB_INFINITY;
            ub[i] = GRB_INFINITY;
        }
        for (int i=n; i<n+n_qc; i++)
        {
            lb[i] = 0.0;
            ub[i] = GRB_INFINITY;
        }
        // Add continuous decision variables and slack variables (one for each qc)
        GRBVar* vars = model.addVars(lb, ub, NULL, NULL, NULL, n+n_qc);

        model.update();

        // Setup of cost function
        GRBQuadExpr obj = 0;
        // First linear part
        for (int i = 0; i < n; i++)
        {
            // f' * x
            if (f(i) != 0)
                obj += f(i)*vars[i];
        }
        for (int i = n; i < n + n_qc; i++)
        {
            obj += weight_fpp*vars[i];
        }
        // Then quadratic part
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (H(i,j) != 0)
                    obj += H(i,j)*vars[i]*vars[j];
            }
        }
        model.setObjective(obj);

        // Setup linear equality constraint
        for (int i = 0; i < A_eq.rows(); i++)
        {
            GRBLinExpr lhs = 0;
            for (int j = 0; j < A_eq.cols(); i++)
            {
                if (A_eq(i,j) != 0)
                    lhs += A_eq(i,j)*vars[j];
            }
            model.addConstr(lhs, GRB_EQUAL, b_eq(i));
        }        

        // Setup linear inequality constraint
        for (int i = 0; i < A_ineq.rows(); i++)
        {
            GRBLinExpr lhs = 0;
            for (int j = 0; j < A_ineq.cols(); j++)
            {
                if (A_ineq(i,j) != 0)
                    lhs += A_ineq(i,j)*vars[j];
            }
            model.addConstr(lhs, GRB_LESS_EQUAL, b_ineq(i));
        }        

        Eigen::MatrixXd Pc_pos[n_qc];
        Eigen::MatrixXd Pc_neg[n_qc];

        GRBQuadExpr qc_base[n_qc];

        // Setup quadratic terms of the quadratic constraints
        for (int i_qc = 0; i_qc < n_qc; i_qc++)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
            es.compute(Pc[i_qc]);

            Eigen::VectorXd lambda = es.eigenvalues();
            Eigen::MatrixXd V = es.eigenvectors();

            Eigen::MatrixXd D_pos = Eigen::MatrixXd::Zero(n, n);
            Eigen::MatrixXd D_neg = Eigen::MatrixXd::Zero(n, n);
            Pc_pos[i_qc] = Eigen::MatrixXd::Zero(n, n);
            Pc_neg[i_qc] = Eigen::MatrixXd::Zero(n, n);
            for (int i=0; i < lambda.size(); i++)
            {
                if (lambda(i) > 0)
                    D_pos(i,i) = lambda(i);    
                else// if (lambda(i) < 0)
                    D_neg(i,i) = lambda(i);    
            }
            Pc_pos[i_qc] = V*D_pos*V.transpose();
            Pc_neg[i_qc] = V*D_neg*V.transpose();
            
            qc_base[i_qc] = 0;
            // Setup the quadratic constraints (the not changing parts)
            // Linear part 
            for (int i=0; i < n; i++)
            {
                qc_base[i_qc] += qc[i_qc](i) * vars[i];
            }
            for (int i=n; i<n+n_qc; i++)
            {
                qc_base[i_qc] += -vars[i];
            }
            // Then quadratic part
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    qc_base[i_qc] += Pc_pos[i_qc](i,j)*vars[i]*vars[j];
/*
                    if (Pc_pos[i_qc](i,j) != 0)
                        qc_base[i_qc] += Pc_pos[i_qc](i,j)*vars[i]*vars[j];
*/
                }
            }
        }
        
        Eigen::VectorXd zm = x0;
        // Now the body of the work, for FPP-SCA
        for (int loop = 1; loop <= max_sca; loop++)
        {
            GRBQConstr qc_constr[n_qc];
            for (int i_qc=0; i_qc<n_qc; i_qc++)
            {
                GRBQuadExpr qc_expr = qc_base[i_qc];
                double qc_var = rc[i_qc] + zm.transpose() * Pc_neg[i_qc] * zm;

                Eigen::VectorXd lin_temp = 2*zm.transpose()*Pc_neg[i_qc];
                // Sets up the non-sca changing things
                for (int i=0; i<n; i++)
                {
                    qc_expr += lin_temp(i)*vars[i];        
/*
                    if (lin_temp(i) != 0)
                        qc_expr += lin_temp(i)*vars[i];        
*/
                }

                std::ostringstream oss;
                oss << "qc" << i_qc;
                qc_constr[i_qc] = model.addQConstr(qc_expr, GRB_LESS_EQUAL, qc_var, oss.str());
            }

            model.optimize();

            double old_OptVal = OptVal;
            OptVal = model.get(GRB_DoubleAttr_ObjVal);
            X_opt = Eigen::VectorXd::Zero(n);
            S_opt = Eigen::VectorXd::Zero(n_qc);
            for (int i=0; i<n; i++)
            {
                X_opt(i) = vars[i].get(GRB_DoubleAttr_X);
            }
            for (int i=0; i<n_qc; i++)
            {
                S_opt(i) = vars[n+i].get(GRB_DoubleAttr_X);
            }
            zm = X_opt;
            
            if (fabs(OptVal - old_OptVal) <= convergence_tol && S_opt.sum() <= convergence_tol)
            {
                std::cout << "Converged. Iterations: " << loop << ". Optimal value: " << OptVal << ". Slack sum: " << S_opt.sum() << std::endl;        
                break;
            }

            // After solving remove the constraints
            for (int i_qc=0; i_qc<n_qc; i_qc++)
            {
                model.remove(qc_constr[i_qc]);
            } 
            model.update();
        }
        
        delete[] vars;

    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl; 
        std::cout << e.getMessage() << std::endl;
    }
    
}
