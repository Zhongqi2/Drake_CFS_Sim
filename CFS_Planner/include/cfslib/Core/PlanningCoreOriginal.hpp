#pragma once
#include "cfslib/Robot/RobotProp.hpp"
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
    InequalityViolated,
    JointStepExceedsLimit,
    None,
    Size
} Status;

class PlanningCoreOriginal
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<PlanningCoreOriginal> Ptr;
        typedef std::shared_ptr<PlanningCoreOriginal const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        // task parameters
        // Eigen::MatrixXd cart_traj_ref_;
        // Eigen::MatrixXd tracking_pt_;
        int execution_time_;
        ////Eigen::MatrixXd obstacles_;

        // cfs parameters
        Eigen::MatrixXd objective_weights_;
        int planning_horizon_;
        int horizon_;
        int num_iter_;
        int dim_;
        double convergence_thresh_;
        double iteration_time_;
        int iteration_;
        double tmp_cost;
        double margin_;
        std::vector<double> qp_time_;
        std::vector<double> process_time_;
        std::vector<double> cost_;
        // Eigen::MatrixXd soln_;
        Eigen::MatrixXd xinit_;
        Eigen::MatrixXd xR_; // xref is the same as xR, by stacking each column of xR;
        Eigen::MatrixXd xref_;
        Eigen::MatrixXd xori_;
        int nstate_;
        int nu_;
        int neq_;
        int nstep_;

        // objective parameters 
        std::vector<double> weight_ref_;
        std::vector<double> weight_self_;

        // robot parameters 
        int njoint_;
        Eigen::MatrixXd base_;

        // obstacle parameters
        Eigen::MatrixXd obs_static_;
        std::vector<Eigen::MatrixXd>  obs_dynamic_;
        int nobs_;
        bool static_obs_;

        // temporal optimization 
        Eigen::MatrixXd zref_; // the time stamps trajectory variable
        double amax_; // the maximum acceleration constraints 
        // Eigen::MatrixXd soln_tempopt_;
        Eigen::MatrixXd xold_tempopt_;
        Eigen::MatrixXd xori_tempopt_;

        // TODO: Ineq & Eq function handler
        std::function<void(const std::vector<Eigen::MatrixXd>& args, std::vector<cfslib::math::Capsule>& caps)> dist_func;

    public:
        Eigen::MatrixXd soln_tempopt_;
        Eigen::MatrixXd soln_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:


    public:
        PlanningCoreOriginal();
        ~PlanningCoreOriginal(){}

        // setter

        // task getter
        // Eigen::MatrixXd cartesian_traj_ref(){return cart_traj_ref_;};
        // Eigen::MatrixXd tracking_point(){return tracking_pt_;};
        int execution_time(){return execution_time_;};
        ////Eigen::MatrixXd obstacles(){return obstacles_;};

        // cfs getter
        Eigen::MatrixXd objective_weights(){return objective_weights_;};
        int planning_horizon(){return planning_horizon_;};

        // cfs operations
        void CoreParse(const robot::Robot::ConstPtr& robot_ptr, const query::ProcessedQuery::ConstPtr& processed_query_ptr);
        void LoadConfig(std::string fname);

        // cfs for path planning 
        void LoadParams();
        int SetCostMatrix3d(Eigen::MatrixXd& H, Eigen::MatrixXd& f);
        int LinConstraint(const robot1::Robot& robot, Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final);
        void EqConstraint(Eigen::MatrixXd& Aeq, Eigen::MatrixXd& beq);
        bool CheckInterval(Eigen::MatrixXd x, const Eigen::MatrixXd& dx_limit);
        Status Iteration(const int& iterMax, const double& tolX,
            const int& resampleMax, const Eigen::MatrixXd& dx_limit,
            robot1::Robot& robot);

        // temporal optimization 
        int SetCostMatrix3dTempopt(Eigen::MatrixXd& H, Eigen::MatrixXd& f);
        int LinConstraintTempopt(Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final);
        int IterationTempopt(int iterMax, double tolX, robot1::Robot& robot);
    
    public:

        void Solve(const robot::Robot::ConstPtr& robot_ptr,
                   const query::ProcessedQuery::ConstPtr& processed_query_ptr);

};
}
}