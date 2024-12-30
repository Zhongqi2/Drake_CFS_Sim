#pragma once
#include "cfslib/Robot/Robot.hpp"
#include "cfslib/Trajectory/Trajectory.hpp"
#include "cfslib/Query/ProcessedQuery.hpp"

namespace cfslib
{
namespace core
{

typedef enum : int {
    OK,
    CFSNotConverged,
    CFSLinNotConverged,
    InequalityViolated,
    JointStepExceedsLimit,
    LogicError,
    None,
    Size
} Status;

const std::vector<std::string> StatusStr = {
    "OK",
    "CFSNotConverged",
    "CFSLinNotConverged",
    "InequalityViolated",
    "JointStepExceedsLimit",
    "LogicError",
    "None",
    "Size"
};

class PlanningCore
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<PlanningCore> Ptr;
        typedef std::shared_ptr<PlanningCore const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        // ! tmp variables, to handle via core parse
        std::vector<Eigen::MatrixXd>  obs_dynamic_; // todo remove
        bool dynamic_obstacle_; // todo remove
        Eigen::MatrixXd l_, s_;

        // problem
        // horizon is number of actions, trajectory has length n_pt = horizon+1
        int horizon_, n_pt_;
        double exec_time_;
        double dt_tempopt_;
        double safety_margin_;
        double amax_ = 0.6;
        int xdim_;
        bool enable_tempopt_;

        // solver
        std::vector<double> weight_ref_;
        std::vector<double> weight_self_;
        int max_iter_, eq_max_iter_;
        double convergence_thres_, eq_thres_;
        bool enforce_eq_;

    /* -------------------------------------------------------------------------- */
    /*                              function pointers                             */
    /* -------------------------------------------------------------------------- */
    private:

        // cfs
        //std::function<double(const Eigen::VectorXd& x, const double& t)> DistFn;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd& x, const double& t, const int robot_link_idx, const int obs_idx)> CentralDiffCapFn;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd& x, const double& t, const int robot_link_idx, const int obs_idx)> CentralDiffBBFn;
        std::function<void(Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final, const Eigen::VectorXd& xx)> LinConstraintFn;
        std::function<void(Eigen::MatrixXd& H, Eigen::MatrixXd& f, const Eigen::VectorXd& xx)> SetCostMatrix3dFn;
        std::function<void(Eigen::MatrixXd& Aeq, Eigen::MatrixXd& beq, const Eigen::VectorXd& xx)> EqConstraintFn;
        std::function<bool(const Eigen::VectorXd& xx)> EqSatisfied;

        // cfs temptop
        std::function<void(Eigen::MatrixXd& H, Eigen::MatrixXd& f)> SetCostMatrix3dTempoptFn;
        std::function<void(Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final,
                                    const Eigen::VectorXd& tref, const Eigen::MatrixXd& xref_mat)> LinConstraintTempoptFn;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:

        void CoreParse(const robot::Robot::ConstPtr& robot_ptr,
                    const query::ProcessedQuery::ConstPtr& processed_query_ptr,
                    Eigen::MatrixXd& xref);
        
        /* ----------------------------- main procedures ---------------------------- */
        Status CFS(Eigen::MatrixXd& xref);
        Status CFSTempOpt(Eigen::VectorXd& tref, const Eigen::MatrixXd& xref);
        // todo add CFSEq

    public:
        PlanningCore();
        ~PlanningCore(){}

    public:
        Eigen::MatrixXd get_l() {return l_;};
        Eigen::MatrixXd get_s() {return s_;};
        Status Solve(
            const robot::Robot::ConstPtr& robot_ptr,
            const query::ProcessedQuery::ConstPtr& processed_query_ptr,
            Eigen::MatrixXd& xref, Eigen::VectorXd& tref);

};

}
}