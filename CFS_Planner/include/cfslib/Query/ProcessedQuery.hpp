#pragma once
#include "cfslib/Utils/Math.hpp"
#include "cfslib/Utils/FileIO.hpp"
#include "cfslib/Utils/CartLQR.hpp"
#include "cfslib/Utils/ICOPInvKine.hpp"
#include "cfslib/Query/UserQuery.hpp"

namespace cfslib
{
namespace query
{
class ProcessedQuery
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:

        typedef std::shared_ptr<ProcessedQuery> Ptr;
        typedef std::shared_ptr<ProcessedQuery const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        /* ------------------------------- task config ------------------------------ */
        Mode mode_;
        cfslib::math::Vector6d x0_;
        cfslib::math::VectorJd q0_;
        Eigen::MatrixXd cart_traj_ref_;
        Eigen::Matrix4d ref_frame_;
        Eigen::Vector3d ref_frame_vel_; // velocity of ref frame in base frame
        Eigen::MatrixXd joint_traj_ref_;
        std::vector<int> critical_ids_;
        int horizon_;
        double execution_time_ = 5.0;
        double cart_vel_max_;
        double cart_acc_max_;
        std::vector<cfslib::math::Capsule> obstacle_capsules_;
        std::vector<cfslib::math::BoundingBox> obstacle_bounding_boxes_;
        Eigen::MatrixXi collision_avoidance_links_;

        /* ---------------------------------- robot --------------------------------- */
        int n_joint_;
        Eigen::MatrixXd DH_;

        /* ------------------------------- hyper param ------------------------------ */
        double traj_hz_ = 10.0;
        double safety_margin_ = 1.0;
        // CFS
        std::vector<double> weight_ref_ {1, 10, 0};
        std::vector<double> weight_self_ {0, 0, 20};
        int cfs_max_iter_, cfs_eq_max_iter_;
        double cfs_convergence_thres_, cfs_eq_thres_;
        int cfs_resample_n_insert_;
        int cfs_max_resample_;
        // ICOP
        double icop_thresh_  = 0.0001;
        int    icop_maxiter_ = 20;
        // CART LQR
        double lqr_Qslack_;
        std::vector<double> Qdiag_, Sdiag_, Rdiag_;
        Eigen::MatrixXd lqr_Q_, lqr_S_, lqr_R_;
        Eigen::VectorXd lqr_bu_, lqr_bxdot_;
        int lqr_horizon_;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        void LoadConfig(std::string fname);
        void ConfigLQR();
        void Print();
        void ProcessGoal(const query::UserQuery::ConstPtr& user_query_ptr, const double& traj_hz);
        // void WeightInit();
        // void SetPlanningHorizon();
        // void set_objective_3d(Eigen::MatrixXd& H, Eigen::MatrixXd& f);
        // void load_Xref_position(std::vector<double>& par);
        // void set_reference_trajectory(std::vector<double>& ref_vec, Eigen::MatrixXd& ref);
        // void set_obj_value(quadprogpp::Matrix<double> &G, quadprogpp::Vector<double> &g0, const Eigen::MatrixXd& HT, const Eigen::MatrixXd& f);
        // void set_cost_matrix_3d(std::vector<std::vector<double> >& Qref, std::vector<std::vector<double> >& Qself);

    public:
        ProcessedQuery();
        ~ProcessedQuery(){}
        
        // setter
        void set_joint_traj_ref(const Eigen::MatrixXd& ref) {joint_traj_ref_ = ref;}
        void set_cart_traj_ref(const Eigen::MatrixXd& ref) {cart_traj_ref_ = ref;}
        void set_trajectory_frequency(const double& hz) {traj_hz_ = hz;}
        void set_execution_time(const double t) {execution_time_ = t;}
        
        // getter
        Mode mode()                         const {return mode_;}
        cfslib::math::Vector6d x0()         const {return x0_;}
        cfslib::math::VectorJd q0()         const {return q0_;}
        Eigen::MatrixXd cart_traj_ref()     const {return cart_traj_ref_;}
        Eigen::MatrixXd joint_traj_ref()    const {return joint_traj_ref_;}
        std::vector<int> critical_ids()     const {return critical_ids_;}
        int     horizon()                   const {return horizon_;}
        double     execution_time()            const {return execution_time_;}
        double  cart_vel_max()              const {return cart_vel_max_;}
        double  cart_acc_max()              const {return cart_acc_max_;}
        std::vector<cfslib::math::Capsule> obstacles_cap()  const {return obstacle_capsules_;} 
        std::vector<cfslib::math::BoundingBox> obstacles_box()  const {return obstacle_bounding_boxes_;} 
        double  trajectory_frequency()      const {return traj_hz_;}
        double  safety_margin()             const {return safety_margin_;}
        std::vector<double> weight_ref()    const {return weight_ref_;}
        std::vector<double> weight_self()   const {return weight_self_;}
        int cfs_max_iter()                  const {return cfs_max_iter_;}
        double cfs_convergence_thres()      const {return cfs_convergence_thres_;}
        int cfs_resample_n_insert()         const {return cfs_resample_n_insert_;}
        int cfs_max_resample()              const {return cfs_max_resample_;}
        int cfs_eq_max_iter()               const {return cfs_eq_max_iter_;}
        double cfs_eq_thres()                  const { std::cout << "----- " << cfs_eq_thres_ << "\n"; return cfs_eq_thres_;}
        Eigen::MatrixXi collision_avoidance_links() const {return collision_avoidance_links_;};

        // Operations
        void Process(const query::UserQuery::ConstPtr& user_query_ptr, const double& execution_time=-1.0);
};
}
}
