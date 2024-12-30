#pragma once
#include "cfslib/Utils/Math.hpp"
#include "QuadProg++.hh"

namespace cfslib
{
namespace math
{

class CartesianLQR2ndOrder
{

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        // specified
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;

        Eigen::VectorXd bu;
        Eigen::VectorXd bv;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd S;
        const double Qs;
        Eigen::MatrixXd R;
        double dt;
        int N;

        const int ud, xd, sd, cartd;

        // composed
        Eigen::MatrixXd Bbar;
        Eigen::MatrixXd Qbar;
        Eigen::MatrixXd Rbar;
        Eigen::MatrixXd Afbar;
        Eigen::MatrixXd Lubar;
        Eigen::MatrixXd bubar;
        Eigen::MatrixXd IRbar;
        Eigen::MatrixXd Ivbar;
        Eigen::MatrixXd bvbar;

        quadprogpp::Matrix<double> G, CE, CI;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:

        void Setup();

    public:
        
        CartesianLQR2ndOrder(
            const Eigen::VectorXd& bu,
            const Eigen::VectorXd& bv,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& S,
            const double& Qslack,
            const Eigen::MatrixXd& R,
            const double& dt, const int& N);
        ~CartesianLQR2ndOrder(){};

        void Config(
            const Eigen::VectorXd& bu,
            const Eigen::VectorXd& bv,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& S,
            const Eigen::MatrixXd& R,
            const double& dt,
            const int& N);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Solve(const Eigen::VectorXd& xk);

};

}

}