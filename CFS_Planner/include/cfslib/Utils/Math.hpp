#pragma once
#include "cfslib/Utils/Common.hpp"
#include "cfslib/Utils/ErrorHandling.hpp"
#include "QuadProg++.hh"

#define PI 3.141592653589793
#define CARTESIAN_DIMS 6
#define TRANS_DIMS 3
#define AXIS_DIMS 3
#define ORIENTATION_DIMS 3
#define DEG2RAD(angle) (static_cast<double>(angle)*M_PI/180.0)
#define RAD2DEG(angle) (static_cast<double>(angle)/M_PI*180.0)

#define N_JOINTS 6 // ruic: for now 6 joints

namespace cfslib
{
namespace math
{

/* -------------------------------------------------------------------------- */
/*                             Vector definitions                             */
/* -------------------------------------------------------------------------- */
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorJd;

/* -------------------------------------------------------------------------- */
/*                                   Matrix                                   */
/* -------------------------------------------------------------------------- */
Eigen::MatrixXd PInv(const Eigen::MatrixXd& M);
Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);
Eigen::MatrixXd EigenHcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);
Eigen::VectorXd ColWiseFlatten(const Eigen::MatrixXd& mat);
Eigen::MatrixXd ColWiseUnflatten(const Eigen::VectorXd& vec, const int& nrow);

/**
 * @brief Return an [N, T+1] matrix linear interpolating from a to b
 * 
 * @param a 
 * @param b 
 * @param T 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd LinInterp(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const int& T);
/**
 * @brief Insert n points between mat(:, i) and mat(:, i+1) using linear interpolation
 * 
 * @param mat 
 * @param i 
 * @param n 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd LinColExpand(const Eigen::MatrixXd& mat, const int& i, const int& n);

/**
 * @brief Cross product of each col of mat and vec
 * 
 * @param mat 
 * @param vec 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd BatchCross(const Eigen::Matrix<double, 3, Eigen::Dynamic>& mat, const Eigen::Vector3d& vec);


Eigen::Matrix3d skew(const Eigen::Vector3d& vec);

/* -------------------------------------------------------------------------- */
/*                         QuadProg++ (to be deleted)                         */
/* -------------------------------------------------------------------------- */
void SetEigenMatFromQuad(Eigen::VectorXd& emat, const quadprogpp::Vector<double>& qvec);
void SetQuadMatFromEigen(quadprogpp::Matrix<double>& qmat, const Eigen::MatrixXd& emat);
void SetQuadVecFromEigen(quadprogpp::Vector<double>& qvec, const Eigen::MatrixXd& evec);

/* -------------------------------------------------------------------------- */
/*                               Transformation                               */
/* -------------------------------------------------------------------------- */
template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data);
void TransMatToPoseEulerRad(const Eigen::Matrix4d& mat, Vector6d& euler);
void TransMatToPoseEulerDeg(const Eigen::Matrix4d& mat, Vector6d& euler);
void TransMatToPoseAngRad(const Eigen::Matrix4d& mat, Vector6d& p);
void TransMatToPoseAngDeg(const Eigen::Matrix4d& mat, Vector6d& p);
void PoseAngRadToTransMat(const Vector6d& p, Eigen::Matrix4d& mat);
void PoseEulerRadToTransMat(const Vector6d& euler, Eigen::Matrix4d& mat);
// Computes p0->p1 transformation IN BASE FRAME
Vector6d GetTransInBase(const Vector6d& p1, const Vector6d& p0);
void BatchApplyTrans(Eigen::MatrixXd& p_arr, const Vector6d& target);
Vector6d LeftDiff(const Vector6d& p0, const Vector6d& p1, const bool& reverse=false);
Vector6d LeftApply(const Vector6d& p0, const Vector6d& diff);

/* -------------------------------------------------------------------------- */
/*                                 Kinematics                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 
 * 
 * @param q [n_joint,]
 * @param DH [1 + n_joint + n_ee, 4]
 * @return Eigen::Matrix4d 
 */
Eigen::Matrix4d FKine(const VectorJd& q, const Eigen::MatrixXd& DH);

/**
 * @brief Overloaded version when link_transform is pre-computed
 * 
 * @param q
 * @param link_transform
 * @return Eigen::Matrix4d 
 */
Eigen::Matrix4d FKine(const VectorJd& q,
    const std::vector<std::function<Eigen::Matrix4d(double q)>>& link_transform);

/**
 * @brief 
 * 
 * @param q [n_joint,]
 * @param pose [6,] target cartesian pose
 * @param DH [1 + n_joint + n_ee, 4]
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd Jacobi(const VectorJd& q, const Vector6d& pose, const Eigen::MatrixXd& DH);

/**
 * @brief Overloaded version when link_transform is pre-computed
 * 
 * @param q 
 * @param pose 
 * @param link_transform 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd Jacobi(const VectorJd& q, const Vector6d& pose,
    const std::vector<std::function<Eigen::Matrix4d(double q)>>& link_transform);

Vector6d IK(
    const cfslib::math::Vector6d& c_ref, const cfslib::math::Vector6d& c0,
    const cfslib::math::VectorJd& theta0, const Eigen::MatrixXd& DH,
    const double& Cartesian_thres, const double& convergence_thres,
    const int& max_iter
);

/**
 * @brief Batch apply FKine to joint traj qq [n_joint, n_pt]
 * 
 * @param qq 
 * @param DH 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd BatchFKine(const Eigen::MatrixXd& qq, const Eigen::MatrixXd& DH);

/**
 * @brief Joint axes
 * 
 * @param q 
 * @param pose 
 * @param DH 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd JointAxes(const VectorJd& q, const Eigen::MatrixXd& DH);

/**
 * @brief Joint axes when link_transforms are available
 * 
 * @param q 
 * @param pose 
 * @param link_transform 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd JointAxes(const VectorJd& q,
    const std::vector<std::function<Eigen::Matrix4d(double q)>>& link_transform);

/* -------------------------------------------------------------------------- */
/*                             Capsule Geometry                               */
/* -------------------------------------------------------------------------- */
struct Capsule{
    Eigen::MatrixXd p; // 3x2
    double r;

    Eigen::Vector3d vel;

    Capsule at(const double& t) const
    {
        Capsule newcap = *this;
        newcap.p.col(0) += vel*t;
        newcap.p.col(1) += vel*t;

        return newcap;
    }
};

struct BoundingBox{
    // All 3x1
    Eigen::Vector3d n1;
    Eigen::Vector3d p1_1;
    Eigen::Vector3d p1_2;

    Eigen::Vector3d n2;
    Eigen::Vector3d p2_1;
    Eigen::Vector3d p2_2;

    Eigen::Vector3d n3;
    Eigen::Vector3d p3_1;
    Eigen::Vector3d p3_2;

    Eigen::Vector3d vel;

    BoundingBox at(const double& t) const
    {
        BoundingBox newbox = *this;
        Eigen::VectorXd dp = vel*t;
        newbox.p1_1 += dp;
        newbox.p1_2 += dp;
        newbox.p2_1 += dp;
        newbox.p2_2 += dp;
        newbox.p3_1 += dp;
        newbox.p3_2 += dp;

        return newbox;
    }
};

typedef struct{
    Eigen::MatrixXd p1; // 3x1
    Eigen::MatrixXd p2; // 3x1
} lineseg;

template <typename T>
T Clip(const T& val, const T& minval, const T& maxval)
{
    if (val < minval)
    {
        return minval;
    }
    else if (val > maxval)
    {
        return maxval;
    }
    else
    {
        return val;
    }
}

/* -------------------------------------------------------------------------- */
/*                              Collision Check                               */
/* -------------------------------------------------------------------------- */
double DistLineSeg(const lineseg& line1, const lineseg& line2);
double DistCap2Cap(const Capsule& cap1, const Capsule& cap2);
double DistCap2BB(const Capsule& cap, const BoundingBox& bb);

/* -------------------------------------------------------------------------- */
/*                             Optimization Helper                            */
/* -------------------------------------------------------------------------- */
bool ConstraintsSatisfied(const Eigen::MatrixXd& x, const Eigen::MatrixXd A, const Eigen::MatrixXd& b,
    const Eigen::MatrixXd& Aeq, const Eigen::MatrixXd& beq);

/* -------------------------------------------------------------------------- */
/*                            Common Operations                               */
/* -------------------------------------------------------------------------- */
bool ApproxEq(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const double& thres);
bool ApproxEqNum(const double& a, const double& b, const double& thres);
bool VecPerp(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);
// std::vector<Capsule> RobotCapPos(const Eigen::MatrixXd& DH, const std::vector<Capsule>& RoCap);

std::string print_eigen_mat(const Eigen::MatrixXd& mat);

/* -------------------------------------------------------------------------- */
/*                              IK Fast                                      */
/* -------------------------------------------------------------------------- */
Eigen::Matrix4d FKine(const Eigen::VectorXd &q, const std::string &robot_model);
cfslib::math::Vector6d IKine(
            const cfslib::math::VectorJd &theta0, const Eigen::Matrix4d &mat_ref,
            const std::string &robot_model,
            const double &Cartesian_thres, const double &convergence_thres,
            const int &max_initializations);
}
}