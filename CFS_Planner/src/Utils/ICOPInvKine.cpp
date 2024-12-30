#include "cfslib/Utils/ICOPInvKine.hpp"

namespace cfslib
{
namespace math
{

/* -------------------------------------------------------------------------- */
/*                                ICOP helpers                                */
/* -------------------------------------------------------------------------- */

void setObjValue(quadprogpp::Matrix<double> &G, quadprogpp::Vector<double> &g0,
                const Eigen::MatrixXd& HT, const Eigen::MatrixXd& f){
    int Hn_, fn_;
    Hn_ = HT.rows();
    fn_ = f.rows();
    // Hfull
    G.resize(Hn_, Hn_);
    for (int i = 0; i < Hn_; i++) 
        for (int j = 0; j < Hn_; j++)
            G[i][j] = HT(i,j);
    // f
    g0.resize(fn_);
    for (int i = 0; i < fn_; i++){
      g0[i] = f(i,0);
    }
        
}

void setConstraint(quadprogpp::Matrix<double> &CI, quadprogpp::Vector<double> &ci0,
                    const Eigen::MatrixXd& LT, const Eigen::MatrixXd& S){
    /*
    * push Lfull_ to qp solver
    * be careful that:
    * according to QP solver, CI should be -Lfull_
    */
    int Lnr_ = LT.rows();
    int Lnc_ = LT.cols();
    int Sn_ = S.rows();
    CI.resize(Lnr_, Lnc_);
    for (int i = 0; i < Lnr_; i++) 
        for (int j = 0; j < Lnc_; j++)
            CI[i][j] = LT(i,j);

    // f
    ci0.resize(Sn_);
    for (int i = 0; i < Sn_; i++) 
        ci0[i] = S(i,0);
}

void QPxset(quadprogpp::Vector<double> &x, const Eigen::MatrixXd& xref){
    x.resize(xref.rows());
    for (int i = 0; i < xref.size(); i++) 
        x[i] = xref(i,0);
}

cfslib::math::VectorJd icopOpt(
    const cfslib::math::VectorJd& theta0, const cfslib::math::Vector6d& c1, const Eigen::MatrixXd& DH)
{
    /*
    * the major optimization procedure for ICOP 
    * transfer the Cartesian to joint space
    * ------- input -------
    * theta0: is the current robot joints configuration 
    * robot: robot status information 
    * c1: the target position in Cartesian space
    * ------- return -------
    * optimized robot joint configuration
    */
    // the initial end effector pos (c0) : Cartesian 0
    // Eigen::MatrixXd c0 = ForKine(theta0, robot.DH, robot.base, robot.cap);
    int n_joint{theta0.rows()};
    Eigen::Matrix4d x0_mat = cfslib::math::FKine(theta0, DH);
    cfslib::math::Vector6d c0;
    cfslib::math::TransMatToPoseAngRad(x0_mat, c0);

    // std::cout << ">> x0_mat:\n" << x0_mat << "\n";

    // objective setting
    // quadratic term
    double penalty[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // penalize 1st and 2nd joint 
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_joint, n_joint); // quadratic term 
    for (int jnt=0; jnt<n_joint; ++jnt){
        H(jnt,jnt) = penalty[jnt];
    }
    // linear term
    Eigen::MatrixXd f = -H.transpose()*theta0; // linear term 

    // equality constriants
    // forward kinematics 
    // Jaccobian of fk
    Eigen::MatrixXd Jac, Diff; 
    Jac = cfslib::math::Jacobi(theta0, c0, DH);
    Diff = Jac; // full cartesian
    // equality constraint 
    Eigen::MatrixXd Aeq, beq;
    Aeq = Diff;
    beq = c1 - c0 + Diff*theta0; // Talyor approximation
      
    // solve QP 
    // set the QP objective and linear and quadratic term 
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    // set objective quadratic and linear terms
    setObjValue(G, g0, H, f); // G is quadratic term and f is the linear term 

    // set linear inequality constraints    
    // not applicable 
    CI.resize(n_joint,0);
    ci0.resize(0);

    // set linear equality cosntraints
    if (beq.rows()==0){// when equality is not applicable 
        CE.resize(n_joint,0);
        ce0.resize(0);
    }
    else{
        Eigen::MatrixXd nAeqT = -1*Aeq.transpose(); // negative transpose
        setConstraint(CE, ce0, nAeqT, beq); // set cosntriants
    }    

    QPxset(x, theta0);// set initial value of u;
    double tmp_cost = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    Eigen::VectorXd theta_new;
    cfslib::math::SetEigenMatFromQuad(theta_new, x);

    return theta_new;
}

Eigen::MatrixXd icop_track_procedure(
    const Eigen::MatrixXd& c_arr, const cfslib::math::Vector6d& c0, const cfslib::math::VectorJd& theta0,
    const Eigen::MatrixXd& DH, const double& thres, const int& max_iter)
{
    #ifdef DEBUG_PRINT
        std::cout << "\n ----------------------- ICOP Cartesian2Joint ------------------------- " << "\n";
    #endif

    int n_joint{theta0.rows()};
    cfslib::math::Vector6d c_ref, c = c0;
    cfslib::math::VectorJd theta = theta0;
    cfslib::math::Vector6d diff; // next robot end effector goal, the difference between current pos and target
    Eigen::Matrix4d mat;
    Eigen::MatrixXd theta_arr(n_joint, c_arr.cols());
    
    int repeat_cnt = 0; // detect safe track repeatation
    for (int t=0; t<c_arr.cols(); ++t){
        
        #ifdef DEBUG_PRINT
            std::cout << "---------- time step " << t << " ----------" << "\n";
        #endif

        c_ref = c_arr.col(t);

        #ifdef DEBUG_PRINT
            std::cout << " +++++ New cartesian ref: \n" << c_ref << "\n";
            std::cout << " +++++ Current cartesian: \n" << c << "\n";
        #endif

        // iterative optimization
        diff = c_ref - c;
        repeat_cnt = 0;
        while(diff.norm() > thres){ // if end effector is not clost to target enough or collision exists

            #ifdef DEBUG_PRINT
                std::cout << ">>> New optimization step <<<" << "\n";
                std::cout << "c: \n" << c << "\n";
                std::cout << "Diff: \n" << diff << "\n";
                std::cout << "Diff norm: " << diff.norm() << "\n";
                std::cout << "theta: \n" << theta << "\n";
            #endif

            theta = icopOpt(theta, c_ref, DH);
            // update previous point
            mat = cfslib::math::FKine(theta, DH);
            cfslib::math::TransMatToPoseAngRad(mat, c);
            diff = c_ref - c; // update the difference to next target

            #ifdef DEBUG_PRINT
                std::cout << "New c: \n" << c << "\n";
                std::cout << "New Diff: \n" << diff << "\n";
                std::cout << "New Diff norm: " << diff.norm() << "\n";
                std::cout << "New theta: \n" << theta << "\n";
            #endif

            repeat_cnt++;
            if (repeat_cnt >= max_iter){
                break;
            }
        }
        #ifdef DEBUG_PRINT
            std::cout << " ------ Done tracking cartesian ref: \n" << c_ref << "\n";
            std::cout << " ------ Final cartesian: \n" << c << "\n";
            std::cout << " ------ Final theta: \n" << theta << "\n";
        #endif
        // record the new theta
        theta_arr.col(t) = theta;
    }

    return theta_arr;
}

Eigen::MatrixXd icop_cartesian2joint(
    const Eigen::MatrixXd& c_arr, const cfslib::math::Vector6d& c0,
    const cfslib::math::VectorJd& theta0, const Eigen::MatrixXd& DH,
    const double& thres, const int& max_iter)
{

    /*
    * set the target Cartesian trajectory 
    * x, y, z, rx, ry, rz
    * c_arr size = 6*n
    */
    if (c_arr.rows() != 6)
    {
        std::ostringstream os;
        os << "icop_cartesian2joint: Expected c_arr with 6 rows, got " << c_arr.rows();
        throw std::logic_error(os.str());
        exit(-1);
    }

    /*
    * safe tracking procedure
    */

    return icop_track_procedure(c_arr, c0, theta0, DH, thres, max_iter);
}

}
}