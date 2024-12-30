#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <math.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <utility>
#include <set>
#include <map>

#define INACTIVE -1

namespace qp
{

typedef Eigen::MatrixXd Mat;

typedef enum : int {
    FeasibleSolution,
    InfeasibleProblem,
    InfeasibleSolution,
    Size
} SolverStatus;

typedef struct {
    double constraint_tolerance;
    int max_iter;
    // double optimality_tolerance;
    // int num_restart;
} Config;

template<typename T>
inline double dinf(){return std::numeric_limits<T>::max();}

class PDQP
{
    /**
     * Positive Definite Quadratic Programming
     * 
     * min(x)   1/2 * x^T * H * x + c^T * x
     *   s.t.   A * x <= b
     *          Aeq * x = beq
     *          lb <= x <= ub
     * 
     * H must be PD
     */

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */

    private:

        void SanityCheck(
            const Mat& H, const Mat& c,
            const Mat& A, const Mat& b,
            const Mat& Aeq, const Mat& beq,
            const Mat& lb, const Mat& ub,
            const Mat& x0,
            int& n, int& m, int& meq);

        SolverStatus ApplyEquality(Mat& Aeq, Mat& beq, Mat& x);

        SolverStatus ApplyInequality(
            const Mat& A, const Mat& b,
            const int& n, const int& meq,
            const Config& options,
            Mat& x,
            const Mat& active_set, const std::vector<int>& aset_id_vec);

        SolverStatus LowerObjective(
            const Mat& H, const Mat& c,
            const Mat& A, const Mat& b,
            const int& n, const int& meq,
            const Config& options,
            Mat& x,
            Mat& active_set, std::vector<int>& aset_id_vec);

    public:
        PDQP();
        ~PDQP(){};

        /** 
         * input:
         *          H, c must be non-empty
         * Phase 1:
         *          Ineq, eq:       start from eq, find feasible, phase 2
         *          No-ineq, eq:    start from eq, phase 2
         *          Ineq, no-eq:    start from x0, find feasible, phase 2
         *          No-ineq, no-eq: closed form solution, no phase 2
         * status:
         *          See enum SolverStatus
         * */
        std::pair<Mat, SolverStatus> solve(
            const Mat& H, const Mat& c,
            const Mat& A, const Mat& b,
            const Mat& Aeq, const Mat& beq,
            const Mat& lb, const Mat& ub,
            const Mat& x0, const Config& options, double& objective);

};

}