#include "qp++.hpp"

#if BUILD_TYPE == BUILD_TYPE_DEBUG
    #define DEBUG_PRINT
#else
    #undef DEBUG_PRINT
#endif

namespace qp
{

namespace log
{
    inline void bug(const std::string& msg){
        std::cout << "[qp][Bug] " << msg;
        throw std::logic_error(msg);
    }
    inline void error(const std::string& msg){
        std::cout << "[qp][Error] " << msg;
        throw std::runtime_error(msg);
    }
    inline void warn(const std::string& msg){std::cout << "[qp][Warning] " << msg;}
    inline void info(const std::string& msg){std::cout << "[qp][Info] " << msg;}
    inline void debug(const std::string& msg){std::cout << "[qp][Debug] " << msg;}
}

namespace mathop
{
    inline qp::Mat PInv(const qp::Mat& M)
    {
        auto nrow = M.rows();
        auto ncol = M.cols();
        qp::Mat Minv;

        if (nrow > ncol)
        {
            Minv = ( ( M.transpose() * M ).inverse() ) * M.transpose();
        }
        else if (nrow < ncol)
        {
            Minv = M.transpose() * ( ( M * M.transpose() ).inverse() );
        }
        else
        {
            Minv = M.inverse();
        }

        return Minv;
    }

    inline qp::Mat EigenVcat(const qp::Mat& mat1, const qp::Mat& mat2)
    {
        /** 
         * [mat1; mat2]
         */

        try
        {
            if (mat1.rows() == 0){
                return mat2;
            }
            else if (mat2.rows() == 0){
                return mat1;
            }
            else{
                if (mat1.cols() != mat2.cols())
                {
                    std::ostringstream msg;
                    msg.clear();
                    msg.str("");
                    msg << "[EigenVcat] Expected mat1 and mat2 to have same cols(), got ["
                        << mat1.cols() << "], [" << mat2.cols() << "]\n";
                    log::error(msg.str());
                }
                qp::Mat new_mat(mat1.rows()+mat2.rows(), mat1.cols());
                new_mat << mat1, mat2;
                return new_mat;
            } 
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            throw;
        }
    }

    inline qp::Mat EigenHcat(const qp::Mat& mat1, const qp::Mat& mat2)
    {
        /** 
         * [mat1 mat2]
         */

        try
        {
            if (mat1.cols() == 0){
                return mat2;
            }
            else if (mat2.cols() == 0){
                return mat1;
            }
            else{
                if (mat1.rows() != mat2.rows())
                {
                    std::ostringstream msg;
                    msg.clear();
                    msg.str("");
                    msg << "[EigenHcat] Expected mat1 and mat2 to have same rows(), got ["
                        << mat1.rows() << "], [" << mat2.rows() << "]\n";
                    log::error(msg.str());
                }
                qp::Mat new_mat(mat1.rows(), mat1.cols()+mat2.cols());
                new_mat << mat1, mat2;
                return new_mat;
            } 
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            throw;
        }
    }

    inline qp::Mat GetNullSpace(const qp::Mat& mat)
    {
        // mat is m * n ()
        // return z s.t. mat * z = 0
        try
        {   
            int n{mat.cols()}; // problem dimension

            const qp::Mat& matT(mat.transpose());
            // get rank
            Eigen::FullPivLU<qp::Mat> lu_decomp(matT);
            int r{lu_decomp.rank()};

            if (r == n)
            {
                std::ostringstream msg;
                msg << "[GetNullSpace] Expected active set to have rank < ["
                    << n << "], got rank [" << r << "]\n";
                log::error(msg.str());
            }

            // QR decomposition of mat.T
            // Eigen::HouseholderQR<qp::Mat> qr(matT); // A = QR, non-rank-revealing
            Eigen::ColPivHouseholderQR<qp::Mat> qr(matT); // AP = QR, rank revealing
            qr.compute(matT);

            qp::Mat P(qr.colsPermutation());
            qp::Mat Q(qr.householderQ());
            qp::Mat R(qr.matrixQR().triangularView<Eigen::Upper>());
            qp::Mat Rfull(R.topLeftCorner(r, r)); // full rank R

            #ifdef DEBUG_PRINT
            std::ostringstream msg;
            msg << "[GetNullSpace] \n";
            // msg << qr.matrixQR() << "\n";
            msg << "---- P ----\n" << P << "\n";
            msg << "---- Q ----\n" << Q << "\n";
            msg << "---- R ----\n" << R << "\n";
            msg << "---- Rfull ----\n" << Rfull << "\n";
            msg << "---- Null space ----\n" << Q.rightCols(n-r) << "\n";
            log::debug(msg.str());
            #endif

            return Q.rightCols(n-r);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            throw;
        }
    }

    inline Mat get_dual_qp(const Mat& x, const Mat& active_set, const Mat& H, const Mat& c)
    {
        // return -mathop::PInv(active_set.transpose())*(H*x+c);
        return active_set.transpose().householderQr().solve(-H*x-c);
    }

    inline Mat get_dual_lp(const Mat& active_set, const Mat& c)
    {
        // return -mathop::PInv(active_set.transpose())*c;
        return active_set.transpose().householderQr().solve(-c);
    }
}

namespace helper
{

    /**
     * @brief Select active constraint to drop based on dual variables
     * 
     * @param dualvar 
     * @param meq 
     * @return int 
     */
    inline int SelectActiveConstraintToDrop(const qp::Mat& dualvar, const int& meq)
    {
        #ifdef DEBUG_PRINT
        log::debug("[SelectActiveConstraintToDrop]\n");
        #endif

        double most_neg_val{0.0};
        int active_constr_to_drop{-1};
        for (int i=meq; i<dualvar.rows(); ++i)
        {
            if (dualvar(i, 0) < most_neg_val)
            {
                most_neg_val = dualvar(i, 0);
                active_constr_to_drop = i - meq;
            }
        }

        #ifdef DEBUG_PRINT
        std::ostringstream msg;
        msg << "dualvar: [" << dualvar.transpose() << "] ";
        msg << "most_neg_val: [" << most_neg_val << "] ";
        msg << "active_constr_to_drop: [" << active_constr_to_drop << "] ";
        msg << "meq: [" << meq << "]\n";
        log::debug(msg.str());
        #endif

        return active_constr_to_drop;
    }

    /**
     * @brief Drop active constraint from active set, update ineq active status
     * 
     * @param active_set 
     * @param ineq_active_status 
     * @param active_constr_to_drop 
     * @param meq 
     */
    inline void DropActiveConstraint(
        qp::Mat& active_set, std::vector<int>& ineq_active_status,
        const int& active_constr_to_drop, const int& meq)
    {
        #ifdef DEBUG_PRINT
        log::debug("[DropActiveConstraint]\n");
        #endif

        // if dual var has < 0, drop most negative active ineq
        auto ineq_to_drop_iter = std::find(
            ineq_active_status.begin(), ineq_active_status.end(), active_constr_to_drop);
        if (ineq_to_drop_iter == ineq_active_status.end())
        {
            log::bug("[DropActiveConstraint] active_constr_to_drop not in ineq_active_status.");
        }
        *ineq_to_drop_iter = INACTIVE;

        // udpate active set
        int head_cnt{meq + active_constr_to_drop};
        int tail_cnt{active_set.rows() - meq - active_constr_to_drop - 1};
        active_set = mathop::EigenVcat(active_set.topRows(head_cnt), active_set.bottomRows(tail_cnt));
        for (auto& val:ineq_active_status)
        {
            if (val > active_constr_to_drop) --val;
        }

        #ifdef DEBUG_PRINT
        std::ostringstream msg;
        msg << "After drop:\n";
        msg << "active_set:\n" << active_set << "\n";
        msg << "ineq_active_status: [";
        for (const auto& val:ineq_active_status) msg << val << " ";
        msg << "]\n";
        log::debug(msg.str());
        #endif
    }

    /**
     * @brief Check if solution is feasible
     * 
     * @param x 
     * @param A 
     * @param b 
     * @param options 
     * @return SolverStatus 
     */
    inline SolverStatus CheckSolutionFeasible(
        const Mat& x, const Mat& A, const Mat& b, const Config& options)
    {
        #ifdef DEBUG_PRINT
        log::debug("[CheckSolutionFeasible]\n");
        #endif

        double M{(A*x-b).maxCoeff()};
        if (M <= options.constraint_tolerance)
        {
            // if constraints are valid, solution found
            return SolverStatus::FeasibleSolution;
        }
        else
        {
            // otherwise, optimization not solved 
            // ! change to restart if needed
            return SolverStatus::InfeasibleSolution;
        }
    }
}

PDQP::PDQP()
{

}

std::pair<qp::Mat, SolverStatus> PDQP::solve(
    const qp::Mat& H_, const qp::Mat& c_,
    const qp::Mat& A_, const qp::Mat& b_,
    const qp::Mat& Aeq_, const qp::Mat& beq_,
    const qp::Mat& lb_, const qp::Mat& ub_,
    const qp::Mat& x0_, const Config& options, double& objective)
{

    try
    {
        
        /* -------------------------------------------------------------------------- */
        /*                                Sanity check                                */
        /* -------------------------------------------------------------------------- */
        int n, m, meq; // xdim, #ineq (excluding boundary), #eq
        this->SanityCheck(H_, c_, A_, b_, Aeq_, beq_, lb_, ub_, x0_, n, m, meq);
        
        /* -------------------------------------------------------------------------- */
        /*                               Initialization                               */
        /* -------------------------------------------------------------------------- */

        qp::Mat H(H_), c(c_), A(A_), b(b_), Aeq(Aeq_), beq(beq_), lb(lb_), ub(ub_), x(x0_);

        /* -------------------------------------------------------------------------- */
        /*                          Phase 1: Find feasible x                          */
        /* -------------------------------------------------------------------------- */

        /* ------------------ add bounds to inequality constraints ------------------ */
        A = mathop::EigenVcat(A, -qp::Mat::Identity(n, n));
        A = mathop::EigenVcat(A, qp::Mat::Identity(n, n));
        b = mathop::EigenVcat(b, -lb);
        b = mathop::EigenVcat(b, ub);

        SolverStatus status;
        std::ostringstream msg;

        /* ----------------------- Apply equality constraints ----------------------- */
        if (meq > 0)
        {
            status = this->ApplyEquality(Aeq, beq, x);

            if (status != SolverStatus::FeasibleSolution)
            {
                return std::make_pair(x, status);
            }
            meq = Aeq.rows();
        }

        /* ----------------------------- Init active set ---------------------------- */
        // [?, n], first meq rows are independent equality constraints
        qp::Mat active_set = Aeq; // default set
        // index of ineq constraint in active set, counting after equality constraints
        std::vector<int> ineq_active_status;
        ineq_active_status.resize(A.rows());
        for (int i=0; i<ineq_active_status.size(); ++i)
        {
            ineq_active_status.at(i) = INACTIVE; // default no active ineq
        }

        /* ---------------------- Apply inequality constraints ---------------------- */
        status = this->ApplyInequality(A, b, n, meq, options, x, active_set, ineq_active_status);
        
        if (status != SolverStatus::FeasibleSolution)
        {
            return std::make_pair(x, status);
        }

        #ifdef DEBUG_PRINT
        msg.clear();
        msg.str("");
        msg << "[Phase 1] ================ Feasible point found ================\n";
        msg << "x: [" << x.transpose() << "]\n";
        msg << "active_set:\n" << active_set << "\n";
        msg << "ineq_active_status: [";
        for (auto& val: ineq_active_status) msg << val << " ";
        msg << "]\n";
        log::debug(msg.str());
        #endif

        /* -------------------------------------------------------------------------- */
        /*                    Phase 2: Iteratively lower objective                    */
        /* -------------------------------------------------------------------------- */

        status = this->LowerObjective(H, c, A, b, n, meq, options, x, active_set, ineq_active_status);
        
        if (status != SolverStatus::FeasibleSolution)
        {
            return std::make_pair(x, status);
        }

        objective = 0.5*(x.transpose()*H*x)(0,0) + (c.transpose()*x)(0,0);

        #ifdef DEBUG_PRINT
        msg.clear();
        msg.str("");
        msg << "[Phase 2] ================= Solution found =================\n";
        msg << "x: [" << x.transpose() << "]\n";
        msg << "objective: " << objective << "\n";
        msg << "active_set:\n" << active_set << "\n";
        msg << "ineq_active_status: [";
        for (auto& val: ineq_active_status) msg << val << " ";
        msg << "]\n";
        log::debug(msg.str());
        #endif

        return std::make_pair(x, SolverStatus::FeasibleSolution);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return std::make_pair(x0_, SolverStatus::InfeasibleProblem);
    }
}

inline void PDQP::SanityCheck(
    const qp::Mat& H, const qp::Mat& c,
    const qp::Mat& A, const qp::Mat& b,
    const qp::Mat& Aeq, const qp::Mat& beq,
    const qp::Mat& lb, const qp::Mat& ub,
    const qp::Mat& x0,
    int& n, int& m, int& meq)
{
    try
    {

        // todo presolve

        std::ostringstream msg;
        
        n = H.rows(); // x dimension
        
        if (H.cols() != n)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [H] shape (" << n << ", " << n << "), got (" << H.rows() << ", "<< H.cols() << ")\n";
            log::error(msg.str());
        }

        Eigen::LLT<Mat> H_llt(H);
        if (!H.isApprox(H.transpose()) || H_llt.info() == Eigen::NumericalIssue) {
            log::error("[SanityCheck] H is not positive definite.");
        }

        if (c.rows() != n || c.cols() != 1)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [c] shape (" << n << ", " << 1 << "), got (" << c.rows() << ", "<< c.cols() << ")\n";
            log::error(msg.str());
        }

        m = A.rows();
        meq = Aeq.rows();

        if ((m > 0) && A.cols() != n)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [A] shape (" << "*" << ", " << n << "), got (" << A.rows() << ", "<< A.cols() << ")\n";
            log::error(msg.str());
        }

        if ((m > 0) && (b.rows() != m || b.cols() != 1))
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [b] shape (" << m << ", " << "1" << "), got (" << b.rows() << ", "<< b.cols() << ")\n";
            log::error(msg.str());
        }

        if ((meq > 0) && Aeq.cols() != n)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [Aeq] shape (*, " << n << "), got (" << Aeq.rows() << ", "<< Aeq.cols() << ")\n";
            log::error(msg.str());
        }

        if ((meq > 0) && (beq.rows() != meq || beq.cols() != 1))
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [beq] shape (" << meq << ", " << "1" << "), got (" << beq.rows() << ", "<< beq.cols() << ")\n";
            log::error(msg.str());
        }

        if (lb.rows() != n || lb.cols() != 1)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [lb] shape (" << n << ", " << "1" << "), got (" << lb.rows() << ", "<< lb.cols() << ")\n";
            log::error(msg.str());
        }

        if (ub.rows() != n || ub.cols() != 1)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [ub] shape (" << n << ", " << "1" << "), got (" << ub.rows() << ", "<< ub.cols() << ")\n";
            log::error(msg.str());
        }

        if (x0.rows() != n || x0.cols() != 1)
        {
            msg.clear();
            msg.str("");
            msg << "[SanityCheck] Expected [x0] shape (" << n << ", " << "1" << "), got (" << x0.rows() << ", "<< x0.cols() << ")\n";
            log::error(msg.str());
        }
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        throw;
    }
}

inline SolverStatus PDQP::ApplyEquality(qp::Mat& Aeq, qp::Mat& beq, qp::Mat& x)
{
    /** 
     * Solve linear system using Eigen linear solver.
     * Check consistency of solution post solving instead of checking rank.
     */
    try
    {
        // x = Aeq.colPivHouseholderQr().solve(beq);
        x = Aeq.householderQr().solve(beq);

        if (!beq.isApprox(Aeq*x))
        {
            log::error("[PDQP::solve] Inconsistent equality constraints.");
        }

        #ifdef DEBUG_PRINT
        std::ostringstream msg;
        msg << "[Phase 1] [Eq]\n";
        msg << "---- Aeq ----\n" << Aeq << "\n";
        msg << "---- beq ----\n" << beq << "\n";
        msg << "---- x ----\n" << x << "\n";
        log::debug(msg.str());
        #endif

        return SolverStatus::FeasibleSolution;
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        throw;
    }
}

inline SolverStatus PDQP::ApplyInequality(
    const qp::Mat& A, const qp::Mat& b,
    const int& n, const int& meq,
    const Config& options,
    qp::Mat& x,
    const qp::Mat& active_set, const std::vector<int>& ineq_active_status)
{
    try
    {
        std::ostringstream msg;

        // Get violation
        double M{(A*x-b).maxCoeff()};

        if (M <= options.constraint_tolerance)
        {
            // x is feasible
            return SolverStatus::FeasibleSolution;
        }
        else
        {
            /**
             * Linear programming
             * 
             * y = [x; gamma]
             * 
             * min(y)   gamma
             *   s.t.   [ A   -1]y  <=  b
             *          [-I   -1]y  <= -lb
             *          [ I   -1]y  <=  ub
             *          [ 0   -1]y  <=  0
             * 
             *          [ 0   -1]y  <=  \rho ( \rho = tol * abs(max(A, Aeq)) ) // ! not used for now, duplicated?
             *          [ Aeq  0]y   =  beq (satisifed by x, but needed for active set)
             */

            /* ---------------------------------- init ---------------------------------- */

            qp::Mat y(n+1, 1); // extended state
            y << x, M+1;
            qp::Mat e(n+1, 1); // objective gamma = <e, y>
            e << qp::Mat::Zero(n, 1), 1;

            // extend ineq
            /**
             * Ae = [A -1;
             *       0 -1]
             */
            qp::Mat Ae = mathop::EigenVcat(A, qp::Mat::Zero(1, n));
            Ae = mathop::EigenHcat(Ae, -qp::Mat::Ones(Ae.rows(), 1));
            qp::Mat be = mathop::EigenVcat(b, qp::Mat::Zero(1, 1));

            /* -------------------------- active set management ------------------------- */
            qp::Mat active_set_e(0, 0);
            if (meq > 0)
            {
                active_set_e = mathop::EigenHcat(active_set, qp::Mat::Zero(meq, 1));
            }
            std::vector<int> ineq_active_status_e((int)ineq_active_status.size(), INACTIVE);
            
            /* -------------------------------- iteration ------------------------------- */
            int iter{0};
            qp::Mat Zk; // null space
            while ((iter++) < options.max_iter)
            {
                #ifdef DEBUG_PRINT
                msg.clear();
                msg.str("");
                msg << "[Phase 1] [Ineq] ------------------ Iteration [" << iter << "] ------------------\n";
                log::debug(msg.str());
                #endif

                /* -------------------------- get search direction -------------------------- */
                if (active_set_e.rows() > 0)
                {
                    Zk = mathop::GetNullSpace(active_set_e); // active_set_e * Z = 0
                }
                else
                {
                    Zk = qp::Mat::Identity(n+1, n+1); // any direction
                }
                qp::Mat dy_hat = -Zk * Zk.transpose() * e;

                /* ---------------------------- compute step size --------------------------- */
                double step{dinf<double>()};
                int new_ineq_id{-1};
                for (int i=0; i<Ae.rows(); ++i) // for all extended ineqs
                {
                    // gamma constraint inactive
                    int aset_id = (i < ineq_active_status_e.size()) ? ineq_active_status_e.at(i) : -1;
                    const qp::Mat& Ai(Ae.row(i));
                    const double proj{(Ai*dy_hat)(0,0)};
                    
                    #ifdef DEBUG_PRINT
                    msg.clear();
                    msg.str("");
                    msg << "Ineq id [" << i << "] ";
                    msg << "Active set id [" << aset_id << "] ";
                    msg << "Ai: [" << Ai << "] ";
                    msg << "Ai*dy_hat: [" << (Ai*dy_hat) << "] ";
                    msg << "Proj: [" << proj << "]\n";
                    log::debug(msg.str());
                    #endif

                    // ineq inactive and towards constraint normal
                    if (aset_id < 0 && proj > 0)
                    {
                        const double& bi{be(i, 0)};
                        double new_step{( bi - (Ai*y)(0,0) ) / proj};
                        if (new_step < step)
                        {
                            step = new_step;
                            new_ineq_id = i;
                        }
                    }
                }
                
                if (isinf(step))
                {
                    log::error("[Phase 1] Unbounded problem.");
                }

                /* ------------------------- update y and active set ------------------------ */
                #ifdef DEBUG_PRINT
                msg.clear();
                msg.str("");
                msg << "y: [" << y.transpose() << "] ";
                msg << "dy_hat: [" << dy_hat.transpose() << "] ";
                msg << "step [" << step << "]\n";
                log::debug(msg.str());
                #endif

                y = y + step * dy_hat;
                x = y.topRows(n);
                M = (A*x-b).maxCoeff(); // update violation

                #ifdef DEBUG_PRINT
                msg.clear();
                msg.str("");
                msg << "new y: [" << y.transpose() << "]\n";
                log::debug(msg.str());
                #endif

                /* ---------------------------- update active set --------------------------- */
                // there must be a newly active constraint, either on x or gamma
                if (new_ineq_id < A.rows()) // update only if new constr is on x
                {
                    // update active set
                    active_set_e = mathop::EigenVcat(active_set_e, Ae.row(new_ineq_id));
                    ineq_active_status_e.at(new_ineq_id) = active_set_e.rows() - 1 - meq;
                }

                /* ---------------------------- check feasibility --------------------------- */
                if (abs(y(n, 0)) <= options.constraint_tolerance)
                {
                    if (M > options.constraint_tolerance)
                    {
                        log::bug("[Phase 1] inconsistent gamma value.");
                    }
                    else
                    {
                        return SolverStatus::FeasibleSolution;
                    }
                }
                else
                {
                    // drop active ineq until active set has rank less than n+1
                    
                    #ifdef DEBUG_PRINT
                    msg.clear();
                    msg.str("");
                    msg << "active_set_e:\n" << active_set_e << "\n";
                    log::debug(msg.str());
                    #endif

                    while (true)
                    {
                        Eigen::FullPivLU<qp::Mat> lu_decomp(active_set_e);
                        int r{lu_decomp.rank()};
                        if (r < n+1) // extended active set has rank < n+1
                        {
                            break;
                        }
                        else // extended active set has rank = n+1
                        {
                            #ifdef DEBUG_PRINT
                            msg.clear();
                            msg.str("");
                            msg << "active_set_e rank: [" << r << "] >= " << n+1 << ", drop ineq.\n";
                            log::debug(msg.str());
                            #endif

                            // lambda = -Ak^{-T}*e
                            qp::Mat dualvar(mathop::get_dual_lp(active_set_e, e));
                            int active_constr_to_drop{helper::SelectActiveConstraintToDrop(dualvar, meq)};

                            if (active_constr_to_drop < 0)
                            {
                                // already optimal but violation > tolerance
                                msg.clear();
                                msg.str("");
                                msg << "[Phase 1] Phase 1 minimum constraint violation ["
                                    << M << "/" << y(n, 0) << "] exceeding tolerance ["
                                    << options.constraint_tolerance << "]";
                                log::error(msg.str());
                            }
                            else
                            {
                                helper::DropActiveConstraint(
                                    active_set_e, ineq_active_status_e,
                                    active_constr_to_drop, meq);
                            }
                        }
                    } // end while
                }
            } // end while
        }

        return SolverStatus::InfeasibleProblem;
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        throw;
    }
}

inline SolverStatus PDQP::LowerObjective(
    const qp::Mat& H, const qp::Mat& c,
    const qp::Mat& A, const qp::Mat& b,
    const int& n, const int& meq,
    const Config& options,
    qp::Mat& x,
    qp::Mat& active_set, std::vector<int>& ineq_active_status)
{
    try
    {
        std::ostringstream msg;
        double M{(A*x-b).maxCoeff()};

        if (M > options.constraint_tolerance)
        {
            msg.clear();
            msg.str("");
            msg << "[Phase 2] Infeasible start point.\n";
            log::bug(msg.str());
        }

        /**
         * Iteratively lower objective while maintaining active set
         */
        
        int iter{0};
        double step{-1.0};
        qp::Mat Zk; // null space
        while ((iter++) < options.max_iter)
        {
            #ifdef DEBUG_PRINT
            msg.clear();
            msg.str("");
            msg << "[Phase 2] --------------- Iteration [" << iter << "] ---------------\n";
            log::debug(msg.str());
            #endif

            /* ------------------------- check current solution ------------------------- */
            /**
             * If step < 1, drop ineq until active set rank < n
             */

            qp::Mat dualvar;
            while (true)
            {
                Eigen::FullPivLU<qp::Mat> lu_decomp(active_set);
                int r{lu_decomp.rank()};

                if (r >= n)
                {
                    #ifdef DEBUG_PRINT
                    msg.clear();
                    msg.str("");
                    msg << "active_set rank: [" << r << "] >= " << n << ", drop ineq.\n";
                    log::debug(msg.str());
                    #endif

                    // get dual var, lambda = -Ak^{-T}(Hx+c)
                    dualvar = mathop::get_dual_qp(x, active_set, H, c);
                    int active_constr_to_drop{helper::SelectActiveConstraintToDrop(dualvar, meq)};

                    // if optimal under active set, check constraints
                    if (active_constr_to_drop < 0)
                    {
                        #ifdef DEBUG_PRINT
                        log::debug("Optimal under active set.\n");
                        #endif
                        return helper::CheckSolutionFeasible(x, A, b, options);
                    }
                    else
                    {
                        #ifdef DEBUG_PRINT
                        log::debug("Found ineq to drop.\n");
                        #endif
                        helper::DropActiveConstraint(
                            active_set, ineq_active_status, active_constr_to_drop, meq);
                    }
                }
                else
                {
                    break;
                }

            } // end while drop ineq

            /* -------------------------- get search direction -------------------------- */
            if (active_set.rows() > 0)
            {
                Zk = mathop::GetNullSpace(active_set); // active_set * Z = 0
            }
            else
            {
                Zk = qp::Mat::Identity(n, n); // any direction
            }
            // qp::Mat dk( -Zk*(Zk.transpose()*H*Zk).inverse()*Zk.transpose()*(c+H*x) );
            // Z.T*H*Z is PD
            qp::Mat dk( Zk*( (Zk.transpose()*H*Zk).llt().solve(-Zk.transpose()*(c+H*x)) ) );

            // #ifdef DEBUG_PRINT
            // msg.clear();
            // msg.str("");
            // msg << "H:\n" << H << "\n";
            // msg << "c:\n" << c << "\n";
            // msg << "Hinv:\n" << H.inverse() << "\n";
            // msg << "Hinv*H:\n" << H.inverse()*H << "\n";
            // msg << "H*Hinv:\n" << H*H.inverse() << "\n";
            // Mat x_optim(-H.inverse()*c);
            // msg << "x_optim: [" << x_optim.transpose() << "]\n";
            // msg << "x'Hx: [" << x_optim.transpose()*H*x_optim << "]\n";
            // msg << "x'H: [" << x_optim.transpose()*H << "]\n";
            // msg << "Hx: [" << H*x_optim << "]\n";
            // msg << "c'x: [" << c.transpose()*x_optim << "]\n";
            // msg << "f_optim: [" << 0.5*(x_optim.transpose()*H*x_optim)(0,0)+(c.transpose()*x_optim)(0,0) << "]\n";
            // log::debug(msg.str());
            // #endif

            #ifdef DEBUG_PRINT
            msg.clear();
            msg.str("");
            msg << "dk: [" << dk.transpose() << "]\n";
            log::debug(msg.str());
            #endif

            /* ---------------------------- compute step size --------------------------- */
            step = 1.0; // reset to unity
            int new_ineq_id{-1};
            qp::Mat proj(A*dk);
            for (int i=0; i<A.rows(); ++i)
            {
                const qp::Mat& Ai(A.row(i));
                const double& proj_i{proj(i,0)};

                // ineq inactive and towards constraint normal
                double new_step{-1.0};
                if (ineq_active_status.at(i) < 0 && proj_i > 0)
                {
                    new_step = ( b(i, 0) - (Ai*x)(0,0) ) / proj_i;
                    if (new_step < step)
                    {
                        step = new_step;
                        new_ineq_id = i;
                    }
                }

                #ifdef DEBUG_PRINT
                msg.clear();
                msg.str("");
                msg << "Ineq id [" << i << "] ";
                msg << "Active set id [" << ineq_active_status.at(i) << "] ";
                msg << "Ai: [" << Ai << "] ";
                msg << "Proj: [" << proj_i << "] ";
                msg << "Step: [" << new_step << "]\n";
                log::debug(msg.str());
                #endif
            }

            if (isinf(step))
            {
                log::error("[Phase 2] Unbounded problem.");
            }

            /* ------------------------- update x and active set ------------------------ */
            #ifdef DEBUG_PRINT
            msg.clear();
            msg.str("");
            msg << "x: [" << x.transpose() << "] ";
            msg << "dk: [" << dk.transpose() << "] ";
            msg << "step [" << step << "]\n";
            log::debug(msg.str());
            #endif

            x = x + step * dk;

            #ifdef DEBUG_PRINT
            msg.clear();
            msg.str("");
            msg << "new x: [" << x.transpose() << "]\n";
            log::debug(msg.str());
            #endif

            /* ---------------------------- update active set --------------------------- */
            if (new_ineq_id >= 0)
            {
                active_set = mathop::EigenVcat(active_set, A.row(new_ineq_id));
                ineq_active_status.at(new_ineq_id) = active_set.rows() - 1 - meq;
            }

            #ifdef DEBUG_PRINT
            msg.clear();
            msg.str("");
            msg << "active_set:\n" << active_set << "\n";
            msg << "ineq_active_status: [";
            for (const auto& val:ineq_active_status) msg << val << " ";
            msg << "]\n";
            log::debug(msg.str());
            #endif
            
            /* ------------------------- check current solution ------------------------- */
            /**
             * If step = 1, drop one ineq if dual var < 0
             */
            if (step == 1)
            {
                #ifdef DEBUG_PRINT
                log::debug("Unity step. Check optimality...\n");
                #endif

                // get dual var, lambda = -Ak^{-T}(Hx+c)
                qp::Mat dualvar(mathop::get_dual_qp(x, active_set, H, c));
                int active_constr_to_drop{helper::SelectActiveConstraintToDrop(dualvar, meq)};

                // if optimal under active set, check constraints
                if (active_constr_to_drop < 0)
                {
                    #ifdef DEBUG_PRINT
                    log::debug("Optimization complete.\n");
                    #endif
                    return helper::CheckSolutionFeasible(x, A, b, options);
                }
                else
                {
                    #ifdef DEBUG_PRINT
                    log::debug("Unity step but optimization not complete. Drop active ineq constraint.\n");
                    #endif
                    helper::DropActiveConstraint(
                        active_set, ineq_active_status, active_constr_to_drop, meq);
                }
            }
        } // end while iteration

        return SolverStatus::InfeasibleProblem;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}