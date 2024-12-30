#pragma once
#include "cfslib/Utils/Math.hpp"
#include "QuadProg++.hh"

namespace cfslib
{
namespace math
{

Eigen::MatrixXd icop_cartesian2joint(
    const Eigen::MatrixXd& c_arr, const cfslib::math::Vector6d& c0,
    const cfslib::math::VectorJd& theta0, const Eigen::MatrixXd& DH,
    const double& thres, const int& max_iter);

}
}