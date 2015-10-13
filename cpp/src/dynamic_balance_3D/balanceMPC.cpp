/**
 * \file LMPC_Base.h
 * \author Darwin Lau
 *
 * \brief Base MPC class for LTI (Linear Time Invariant) Systems
 *  The aim of the base class is just to construct S and T matrices within X_k = S U_k + T x_k 
 */

#include "dynamic_balance_3D/balanceMPC.h"
#include <Eigen/Geometry>

#include <iostream>

BalanceMPC::BalanceMPC(const Eigen::VectorXd& _dtVec, double _mass, double _w_CoM, double _w_jerk) :
    LMPC_Base(_generateA_vec(_dtVec), _generateB_vec(_dtVec)), qcqp_solver(QCQP_FPPSCA_Solver(FPPSCA_W_FPP, FFPSCA_MAXSCA, FFPSCA_TOL))
{
    dtVec = _dtVec;
    mass = _mass;
    w_CoM = _w_CoM;
    w_jerk = _w_jerk;

    // Construct the cost function matrices
    Q_CoM = w_CoM * Eigen::MatrixXd::Identity(N_DIMS * N_h, N_DIMS * N_h); // 3 because of x, y, z dimensions
    Q_jerk = w_jerk * Eigen::MatrixXd::Identity(N_DIMS * N_h, N_DIMS * N_h); // 3 because x, y, z dimensions

    // Construct the re-usable matrices
    T_pos = Eigen::MatrixXd::Zero(N_DIMS, N_STATES); // 3 position components from the 9 states
    T_posz = Eigen::MatrixXd::Zero(1, N_STATES); // 1 position component (z) from the 9 states
    T_acc = Eigen::MatrixXd::Zero(N_DIMS, N_STATES); // 3 position components from the 9 states

    for (int i = 0; i < N_DIMS; i++)
    {
        T_pos.block<1, N_LEVELS>(i, i*N_LEVELS) << 1, 0, 0;
        T_acc.block<1, N_LEVELS>(i, i*N_LEVELS) << 0, 0, 1;
    }
    T_posz(Z_IND*N_LEVELS) = 1;

    // X_Gk = T_P X_k
    T_P = Eigen::MatrixXd::Zero(T_pos.rows() * N_h, T_pos.cols() * N_h);
    // X_Gk_z = T_Pz X_k
    T_Pz = Eigen::MatrixXd::Zero(T_posz.rows() * N_h, T_posz.cols() * N_h);
    // X_dd_Gk = T_A X_k
    T_A = Eigen::MatrixXd::Zero(T_acc.rows() * N_h, T_acc.cols() * N_h);
    // T_N to obtain the state at the end of the horizon N
    T_N = Eigen::MatrixXd::Zero(N_STATES, N_STATES * N_h);
    T_N.block<N_STATES,N_STATES>(0, N_STATES*(N_h-1)) = Eigen::MatrixXd::Identity(N_STATES,N_STATES);
    T_N_t = T_N.transpose();

    // Assign those matrices
    for (int i = 0; i < N_h; i++)
    {
        T_P.block(i*T_pos.rows(), i*T_pos.cols(), T_pos.rows(), T_pos.cols()) = T_pos;   
        T_A.block(i*T_acc.rows(), i*T_acc.cols(), T_acc.rows(), T_acc.cols()) = T_acc;   
        T_Pz.block(i*T_posz.rows(), i*T_posz.cols(), T_posz.rows(), T_posz.cols()) = T_posz;   
    }

    H = S.transpose() * T_P.transpose() * Q_CoM * T_P * S +  Q_jerk;

    // Pre-compute the required transposes
    T_acc_t = T_acc.transpose();
    T_P_T = T_P * T;
    Q_CoM_T_P_S = Q_CoM * T_P * S;

    Gamma_G = Eigen::Vector3d(0.0, 0.0, -mass * GRAV_CONST);
    Gamma.resize(N_h, Gamma_G);
}

void BalanceMPC::compute(const MPCModelState& state, const std::vector<MPCModelState>& traj_ref, const Eigen::VectorXd& U_prev)
{
    soln = state;
    soln_full = traj_ref;

    // Linear inequality matrices to pass into the solver
    Eigen::MatrixXd A_ineq = Eigen::MatrixXd();
    Eigen::VectorXd b_ineq = Eigen::VectorXd();

    // X_G reference vector over the horizon
    Eigen::VectorXd X_G_ref = Eigen::VectorXd::Zero(N_DIMS * N_h);
    // Initial state for the horizon
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(N_STATES);
    // Active contacts
    std::vector<ContactState> aContacts;

    // Storing the constraints in X space, X' * P_Xc[i] * X + q_Xc[i]' * X <= r_Xc[i]
    std::vector<Eigen::MatrixXd> P_Xc;
    std::vector<Eigen::VectorXd> q_Xc;
    std::vector<double> r_Xc;
    // Storing the constraints in U space, U' * P_Uc[i] * U + q_Uc[i]' U <= r_Uc[i]
    std::vector<Eigen::MatrixXd> P_Uc;
    std::vector<Eigen::VectorXd> q_Uc;
    std::vector<double> r_Uc;
    // Storing the inequality constraints in 
    std::vector<Eigen::VectorXd> a_ineq_vec;
    std::vector<double> b_ineq_vec;

    // Precompute some matrices once
    Eigen::MatrixXd T_pos_T_i;
    Eigen::MatrixXd T_i_t_T_acc_t;

    // Converts the state object into a vector
    x0 = _stateToX(state);
    // Converts the state within the trajectory reference to a single vector
    for (int i = 0; i < N_h; i++)
    {
        X_G_ref.segment<N_DIMS>(i * N_DIMS) = traj_ref[i].x_G;
    }
    // Setup the linear part of the cost function: U' * H * U + f' * U
    Eigen::VectorXd f;
    f = (2*(T_P_T * x0 - X_G_ref).transpose() * Q_CoM_T_P_S).transpose();

    // Construct the dynamic balance constraints
    for (int i = 0; i < N_h; i++)
    {
        // Setup T_i
        Eigen::MatrixXd T_i = Eigen::MatrixXd::Zero(N_STATES, N_STATES * N_h);
        T_i.block<N_STATES,N_STATES>(0, N_STATES*i) = Eigen::MatrixXd::Identity(N_STATES,N_STATES);
        Eigen::MatrixXd T_i_t = T_i.transpose();

        T_pos_T_i = T_pos * T_i;
        T_i_t_T_acc_t = T_i_t * T_acc_t;

        aContacts = traj_ref[i].getActiveContacts();

        // Setup single contact phase
        if (aContacts.size() == 1)
        {
            ContactState c = aContacts[0];
            Eigen::Vector3d a1 = c.tr();
            Eigen::Vector3d a2 = c.tl();
            Eigen::Vector3d a3 = c.bl();
            Eigen::Vector3d a4 = c.br();

            // Define the temp matrices
            Eigen::MatrixXd P_X1, P_X2, P_X3, P_X4;
            Eigen::VectorXd q_X1, q_X2, q_X3, q_X4;
            double r_X1, r_X2, r_X3, r_X4;
    
            P_X1 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(a1 - a2) * T_pos_T_i; 
            P_X2 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(a2 - a3) * T_pos_T_i; 
            P_X3 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(a3 - a4) * T_pos_T_i; 
            P_X4 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(a4 - a1) * T_pos_T_i; 

            q_X1 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(a1-a2) * T_pos - mass * ((a1-a2).cross(c.centre()) + a1.cross(a2)).transpose() * T_acc).transpose();
            q_X2 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(a2-a3) * T_pos - mass * ((a2-a3).cross(c.centre()) + a2.cross(a3)).transpose() * T_acc).transpose();
            q_X3 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(a3-a4) * T_pos - mass * ((a3-a4).cross(c.centre()) + a3.cross(a4)).transpose() * T_acc).transpose();
            q_X4 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(a4-a1) * T_pos - mass * ((a4-a1).cross(c.centre()) + a4.cross(a1)).transpose() * T_acc).transpose();

            r_X1 = - ((a1-a2).cross(c.centre()) + a1.cross(a2)).transpose() * Gamma[i];
            r_X2 = - ((a2-a3).cross(c.centre()) + a2.cross(a3)).transpose() * Gamma[i];
            r_X3 = - ((a3-a4).cross(c.centre()) + a3.cross(a4)).transpose() * Gamma[i];
            r_X4 = - ((a4-a1).cross(c.centre()) + a4.cross(a1)).transpose() * Gamma[i];

            P_Xc.push_back((P_X1.transpose() + P_X1)/2);
            P_Xc.push_back((P_X2.transpose() + P_X2)/2);
            P_Xc.push_back((P_X3.transpose() + P_X3)/2);
            P_Xc.push_back((P_X4.transpose() + P_X4)/2);

            q_Xc.push_back(q_X1);
            q_Xc.push_back(q_X2);
            q_Xc.push_back(q_X3);
            q_Xc.push_back(q_X4);

            r_Xc.push_back(r_X1);
            r_Xc.push_back(r_X2);
            r_Xc.push_back(r_X3);
            r_Xc.push_back(r_X4);
        }
        // Setup double contact phase
        else if (aContacts.size() == 2)
        {
            double z;
            Eigen::Vector3d x_centre;

            if (aContacts[0].centre().z() >= aContacts[1].centre().z())
                z = aContacts[0].centre().z();
            else
                z = aContacts[1].centre().z();

            x_centre = Eigen::Vector3d(0.0, 0.0, z+100);             

            double triple_cross_product;
            triple_cross_product = (aContacts[1].top() - x_centre).cross(aContacts[1].bottom() - x_centre).transpose() * (aContacts[0].top() - x_centre);

            // Arbitrary assignment
            ContactState c_L = aContacts[0];
            ContactState c_R = aContacts[0];

            // index 0 is "left" contact and index 1 is "right" contact
            if (triple_cross_product > 0)
            {
                c_L = aContacts[0];
                c_R = aContacts[1];
            }
            // index 1 is "left" contact and index 0 is "right" contact
            else if (triple_cross_product < 0)
            {
                c_R = aContacts[0];
                c_L = aContacts[1];
            }
            else
            {
            }
    
            // Define the temp matrices
            Eigen::MatrixXd P_X1, P_X2, P_X3, P_X4;
            Eigen::VectorXd q_X1, q_X2, q_X3, q_X4;
            double r_X1, r_X2, r_X3, r_X4;

            P_X1 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(c_L.t() - c_L.b()) * T_pos_T_i;
            P_X2 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(c_R.b() - c_R.t()) * T_pos_T_i;
            P_X3 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(c_L.centre() - c_R.centre()) * T_pos_T_i;
            P_X4 = mass * T_i_t_T_acc_t * _skewSymmetricMatrix(c_R.centre() - c_L.centre()) * T_pos_T_i;

            q_X1 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(c_L.t()-c_L.b()) * T_pos - mass * ((c_L.top()).cross(c_L.bottom())).transpose() * T_acc).transpose();
            q_X2 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(c_R.b()-c_R.t()) * T_pos - mass * ((c_R.bottom()).cross(c_R.top())).transpose() * T_acc).transpose();
            q_X3 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(c_L.centre()-c_R.centre()) * T_pos - mass * ((c_L.bottom()).cross(c_R.bottom())).transpose() * T_acc).transpose();
            q_X4 = T_i_t * (-Gamma[i].transpose() * _skewSymmetricMatrix(c_R.centre()-c_L.centre()) * T_pos - mass * ((c_R.top()).cross(c_L.top())).transpose() * T_acc).transpose();

            r_X1 = - ((c_L.top()).cross(c_L.bottom())).transpose() * Gamma[i]; 
            r_X2 = - ((c_R.bottom()).cross(c_R.top())).transpose() * Gamma[i]; 
            r_X3 = - ((c_L.bottom()).cross(c_R.bottom())).transpose() * Gamma[i]; 
            r_X4 = - ((c_R.top()).cross(c_L.top())).transpose() * Gamma[i]; 

            P_Xc.push_back((P_X1.transpose() + P_X1)/2);
            P_Xc.push_back((P_X2.transpose() + P_X2)/2);
            P_Xc.push_back((P_X3.transpose() + P_X3)/2);
            P_Xc.push_back((P_X4.transpose() + P_X4)/2);

            q_Xc.push_back(q_X1);
            q_Xc.push_back(q_X2);
            q_Xc.push_back(q_X3);
            q_Xc.push_back(q_X4);

            r_Xc.push_back(r_X1);
            r_Xc.push_back(r_X2);
            r_Xc.push_back(r_X3);
            r_Xc.push_back(r_X4);
        }
    }

    // Turn the P_Xc, q_Xc, r_Xc constraints to U term
    for (int i = 0; i < P_Xc.size(); i++)
    {
        // Constraint in form X^T * P_Xc[i] * X + q_Xc[i]^T X <= r_Xc[i]
        // And since X = S * U + T * k0
        P_Uc.push_back(S_t * P_Xc[i] * S);
        q_Uc.push_back((2 * x0.transpose() * T_t * P_Xc[i] *  S + q_Xc[i].transpose() * S).transpose());
        r_Uc.push_back(r_Xc[i] - x0.transpose() * T_t * P_Xc[i] * T * x0 - q_Xc[i].transpose() * T * x0);
    }

    // Construct the linear acceleration constraints

    // Construct the linear terminal constraint
    if (ENABLE_TERMINAL_CONSTRAINTS)
    {
        aContacts = traj_ref[N_h - 1].getActiveContacts();
        if (aContacts.size() == 1)
        {
            ContactState c = aContacts[0];
            Eigen::Vector3d a1 = c.tr();
            Eigen::Vector3d a2 = c.tl();
            Eigen::Vector3d a3 = c.bl();
            Eigen::Vector3d a4 = c.br();


            // Define the temp matrices
            Eigen::VectorXd q_x1, q_x2, q_x3, q_x4;
            double r_1, r_2, r_3, r_4;

            q_x1 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(a1-a2) * T_pos).transpose();
            q_x2 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(a2-a3) * T_pos).transpose();
            q_x3 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(a3-a4) * T_pos).transpose();
            q_x4 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(a4-a1) * T_pos).transpose();

            r_1 = - ((a1-a2).cross(c.centre()) + a1.cross(a2)).transpose() * Gamma[N_h - 1];
            r_2 = - ((a2-a3).cross(c.centre()) + a2.cross(a3)).transpose() * Gamma[N_h - 1];
            r_3 = - ((a3-a4).cross(c.centre()) + a3.cross(a4)).transpose() * Gamma[N_h - 1];
            r_4 = - ((a4-a1).cross(c.centre()) + a4.cross(a1)).transpose() * Gamma[N_h - 1];

            a_ineq_vec.push_back(S_t * T_N_t * q_x1);
            a_ineq_vec.push_back(S_t * T_N_t * q_x2);
            a_ineq_vec.push_back(S_t * T_N_t * q_x3);
            a_ineq_vec.push_back(S_t * T_N_t * q_x4);

            b_ineq_vec.push_back(r_1 - q_x1.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_2 - q_x2.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_3 - q_x3.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_4 - q_x4.transpose() * T_N * T * x0);
        }
        else if (aContacts.size() == 2)
        {
            double z;
            Eigen::Vector3d x_centre;

            if (aContacts[0].centre().z() >= aContacts[1].centre().z())
                z = aContacts[0].centre().z();
            else
                z = aContacts[1].centre().z();

            x_centre = Eigen::Vector3d(0.0, 0.0, z+100);

            double triple_cross_product;
            triple_cross_product = (aContacts[1].top() - x_centre).cross(aContacts[1].bottom() - x_centre).transpose() * (aContacts[0].top() - x_centre);

            // Arbitrary assignment
            ContactState c_L = aContacts[0];
            ContactState c_R = aContacts[0];

            // index 0 is "left" contact and index 1 is "right" contact
            if (triple_cross_product > 0)
            {
                c_L = aContacts[0];
                c_R = aContacts[1];
            }
            // index 1 is "left" contact and index 0 is "right" contact
            else if (triple_cross_product < 0)
            {
                c_R = aContacts[0];
                c_L = aContacts[1];
            }
            else
            {
            }

            // Define the temp matrices
            Eigen::VectorXd q_x1, q_x2, q_x3, q_x4;
            double r_1, r_2, r_3, r_4;

            q_x1 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(c_L.t()-c_L.b()) * T_pos).transpose();
            q_x2 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(c_R.b()-c_R.t()) * T_pos).transpose();
            q_x3 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(c_L.centre()-c_R.centre()) * T_pos).transpose();
            q_x4 = (-Gamma[N_h - 1].transpose() * _skewSymmetricMatrix(c_R.centre()-c_L.centre()) * T_pos).transpose();

            r_1 = - ((c_L.top()).cross(c_L.bottom())).transpose() * Gamma[N_h - 1];
            r_2 = - ((c_R.bottom()).cross(c_R.top())).transpose() * Gamma[N_h - 1];
            r_3 = - ((c_L.bottom()).cross(c_R.bottom())).transpose() * Gamma[N_h - 1];
            r_4 = - ((c_R.top()).cross(c_L.top())).transpose() * Gamma[N_h - 1];

            a_ineq_vec.push_back(S_t * T_N_t * q_x1);
            a_ineq_vec.push_back(S_t * T_N_t * q_x2);
            a_ineq_vec.push_back(S_t * T_N_t * q_x3);
            a_ineq_vec.push_back(S_t * T_N_t * q_x4);

            b_ineq_vec.push_back(r_1 - q_x1.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_2 - q_x2.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_3 - q_x3.transpose() * T_N * T * x0);
            b_ineq_vec.push_back(r_4 - q_x4.transpose() * T_N * T * x0);
        }
        else if (aContacts.size() == 3)
        {   
        }

        A_ineq = Eigen::MatrixXd::Zero(a_ineq_vec.size(), N_h * N_DIMS);
        b_ineq = Eigen::VectorXd::Zero(a_ineq_vec.size());
        for (int i = 0; i < a_ineq_vec.size(); i++)
        {
            A_ineq.row(i) = a_ineq_vec[i].transpose();
            b_ineq(i) = b_ineq_vec[i];
        }
    }

    // Solve the problem
    qcqp_solver.solve(H, f, Eigen::MatrixXd(), Eigen::VectorXd(), A_ineq, b_ineq, P_Uc, q_Uc, r_Uc, U_prev);

    U_opt = qcqp_solver.X_opt;
    u_opt = U_opt.head(N_DIMS);
    X_opt = S * U_opt + T * x0;
    x_opt = X_opt.head(N_STATES);

    for (int i = 0; i < N_h; i++)
    {
        soln_full[i].x_G_ddd = U_opt.segment(i*N_DIMS, N_DIMS);
        _XToState(X_opt.segment(i*N_STATES, N_STATES), soln_full[i].x_G, soln_full[i].x_G_d, soln_full[i].x_G_dd);
    }
    soln = soln_full[0];
}

Eigen::VectorXd BalanceMPC::_stateToX(const MPCModelState& s)
{
    Eigen::VectorXd X = Eigen::VectorXd::Zero(N_STATES);
    X << s.x_G[X_IND], s.x_G_d[X_IND], s.x_G_dd[X_IND],
        s.x_G[Y_IND], s.x_G_d[Y_IND], s.x_G_dd[Y_IND],
        s.x_G[Z_IND], s.x_G_d[Z_IND], s.x_G_dd[Z_IND];
    return X;
}

void BalanceMPC::_XToState(const Eigen::VectorXd& s, Eigen::Vector3d& x_G, Eigen::Vector3d& x_G_d, Eigen::Vector3d& x_G_dd)
{
    x_G << s[0], s[3], s[6];
    x_G_d << s[1], s[4], s[7];
    x_G_dd << s[2], s[5], s[8];
}

// Generates the state matrix A from x_{k+1} = A x_k + B u_k
Eigen::MatrixXd BalanceMPC::_generateA(double dt)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_STATES,N_STATES);
    A.block<N_LEVELS,N_LEVELS>(0,0) << 1, dt, dt*dt/2, 
                        0, 1, dt, 
                        0, 0, 1;
    A.block<N_LEVELS,N_LEVELS>(3,3) << 1, dt, dt*dt/2, 
                        0, 1, dt, 
                        0, 0, 1;
    A.block<N_LEVELS,N_LEVELS>(6,6) << 1, dt, dt*dt/2, 
                        0, 1, dt, 
                        0, 0, 1;
    return A;
}

// Generates the state matrix B from x_{k+1} = A x_k + B u_k
Eigen::MatrixXd BalanceMPC::_generateB(double dt)
{
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9,3);
    B.block<3,1>(0,0) << dt*dt*dt/6, dt*dt/2, dt; 
    B.block<3,1>(3,1) << dt*dt*dt/6, dt*dt/2, dt; 
    B.block<3,1>(6,2) << dt*dt*dt/6, dt*dt/2, dt; 

    return B;
}

// Generate an array of A matrices for an array of dt
std::vector<Eigen::MatrixXd> BalanceMPC::_generateA_vec(const Eigen::VectorXd& dt)
{
    int N = dt.size();
    std::vector<Eigen::MatrixXd> A(N, Eigen::MatrixXd());

    for (int i = 0; i < N; i++)
    {
        A[i] = _generateA(dt[i]);
    }
    return A;
}

// Generate an array of B matrices for an array of dt
std::vector<Eigen::MatrixXd> BalanceMPC::_generateB_vec(const Eigen::VectorXd& dt)
{
    int N = dt.size();
    std::vector<Eigen::MatrixXd> B(N, Eigen::MatrixXd());

    for (int i = 0; i < N; i++)
    {
        B[i] = _generateB(dt[i]);
    }
    return B;
}

Eigen::Matrix3d BalanceMPC::_skewSymmetricMatrix(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d M;
    M << 0, -v[2], v[1], 
        v[2], 0, -v[0],
        -v[1], v[0], 0;
    return M;
}
