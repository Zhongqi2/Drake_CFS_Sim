#pragma once
#include "cfslib/Core/PlanningCore.hpp"

namespace cfslib
{

class CFSLib
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<CFSLib> Ptr;
        typedef std::shared_ptr<CFSLib const> ConstPtr;
    
    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:


    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        bool JointVelOK(const Eigen::MatrixXd& x, const Eigen::MatrixXd& dx_limit);
        bool JointDispOK(const Eigen::MatrixXd& x, const Eigen::MatrixXd& dx_limit);

    public:

        CFSLib();
        ~CFSLib(){}

        trajectory::Trajectory::ConstPtr Solve(const query::UserQuery::ConstPtr& query);
        trajectory::Trajectory::Ptr DemoSolve(const query::UserQuery::ConstPtr& query);
        query::ProcessedQuery::Ptr TmpSolve(const query::UserQuery::ConstPtr& query);
};

}