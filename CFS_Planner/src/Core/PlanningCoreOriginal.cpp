#include "cfslib/Core/PlanningCoreOriginal.hpp"

namespace cfslib
{

namespace core
{

namespace helper
{

struct lineseg{
    Eigen::MatrixXd p1;
    Eigen::MatrixXd p2;
};

struct inputfield{
    std::string field;
    std::vector<float> elements;
};


/* -------------------------------------------------------------------------- */
/*                              Auxiliary functions                           */
/* -------------------------------------------------------------------------- */

void quad2matrix(quadprogpp::Vector<double> u, Eigen::MatrixXd& u_){
    int row = u_.rows();
    for(int i=0; i<row; ++i){
        u_(i,0) = u[i];
    }
}    

void setObjValue(quadprogpp::Matrix<double> &G, quadprogpp::Vector<double> &g0, const Eigen::MatrixXd& HT, const Eigen::MatrixXd& f){

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
    for (int i = 0; i < fn_; i++) 
        g0[i] = f(i,0);
}

void setConstraint(quadprogpp::Matrix<double> &CI, quadprogpp::Vector<double> &ci0, const Eigen::MatrixXd& LT, const Eigen::MatrixXd& S){
/* push Lfull_ to qp solver
    be careful that:
    according to QP solver, CI should be -Lfull_*/
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

bool checkConvergence(const Eigen::MatrixXd& x, Eigen::MatrixXd& xold, const double& tolX){
    Eigen::MatrixXd diff;
    diff = x - xold;
    double norm_diff;
    norm_diff = diff.norm();
    if (norm_diff < tolX){
        return true;
    }
    else{
        return false;
    }
}

bool checkLocalOpt(const Eigen::MatrixXd& L, const Eigen::MatrixXd& S, const Eigen::MatrixXd& x){
    bool optima = true;
    Eigen::MatrixXd diff;
    diff = S - L*x;
    for (int i=0; i<diff.rows(); ++i){
        if (diff(i,0) <= 0){
            optima = false;
            break;
        }
    }
    return optima;
}

void QPxset(quadprogpp::Vector<double> &x, const Eigen::MatrixXd& xref){
    x.resize(xref.rows());
    for (int i = 0; i < xref.size(); i++) 
        x[i] = xref(i,0);
}

double pow_sum(Eigen::MatrixXd m, int pwr){
    assert(m.cols() == 1);
    double sum = 0;
    Eigen::MatrixXd tmp;
    tmp = m.array().pow(pwr);
    for (int i=0; i<m.rows(); ++i){
        sum += tmp(i,0);
    }
    return sum;
}

double dot_sum(Eigen::MatrixXd m, Eigen::MatrixXd n){
    assert(m.rows() == n.rows());
    double sum = 0;
    for (int i=0; i<m.rows(); ++i){
        sum += m(i,0) * n(i,0);
    }
    return sum;
}

double fixbound(double num){
    if (num < 0){
        num = 0;
    }
    else{
        if (num > 1){
        num = 1;
        }
    }
    return num;
}

double distLinSeg(Eigen::MatrixXd s1, Eigen::MatrixXd e1, Eigen::MatrixXd s2, Eigen::MatrixXd e2){
    Eigen::MatrixXd d1, d2, d12;
    d1 = e1 - s1;
    d2 = e2 - s2;
    d12 = s2 - s1;
    
    double D1 = pow_sum(d1, 2);
    double D2 = pow_sum(d2, 2);
    double S1 = dot_sum(d1, d12);
    double S2 = dot_sum(d2, d12);
    double R = dot_sum(d1, d2);
    double den = D1*D2 - pow(R,2);

    //if one of the segments is a point
    double u,t,uf;
    if (D1 == 0 || D2 == 0){
        if (D1 != 0){
        u = 0;
        t = S1 / D1;
        t = fixbound(t);
        }
        else if (D2 != 0){ 
        t = 0;
        u = -S2 / D2;
        u = fixbound(u);
        }
        else{ // both line segment is point 
        t = 0;
        u = 0;
        }
    }

    else if (den == 0){ // line is parallel 
        t = 0;
        u = -S2/D2;
        uf = fixbound(u);
        if (uf != u){
        t = (uf*R+S1)/D1;
        t = fixbound(t);
        u = uf;
        }
    }

    else{ // general case
        t = (S1*D2-S2*R)/den;
        t = fixbound(t);
        u = (t*R-S2)/D2;
        uf = fixbound(u);
        if (uf != u){
        t = (uf*R+S1)/D1;
        t = fixbound(t);
        u = uf;
        }
    }

    // compute the distance, given t and u 
    Eigen::MatrixXd distm;
    distm = d1*t-d2*u-d12;
    double dist = distm.norm();

    // point
    Eigen::MatrixXd point(3,2);
    point.block(0,0,3,1) = s1 + d1*t;
    point.block(0,1,3,1) = s2 + d2*u;
    return dist;
}

void CapPos(
    const Eigen::MatrixXd& base, const Eigen::MatrixXd& DH,
    std::vector<cfslib::math::Capsule>& RoCap, std::vector<Eigen::MatrixXd>& M, std::vector<lineseg>& pos)
{
    int nlink = DH.rows();
    Eigen::MatrixXd R, T, JTR;
    M[0].resize(4,4);
    M[0] << Eigen::MatrixXd::Identity(3,3), base,
            0, 0, 0, 1;

    for (int i=0; i<nlink; ++i)
    {
        R.resize(3,3);
        R << cos(DH(i,0)), -sin(DH(i,0))*cos(DH(i,3)), sin(DH(i,0))*sin(DH(i,3)),
            sin(DH(i,0)), cos(DH(i,0))*cos(DH(i,3)), -cos(DH(i,0))*sin(DH(i,3)),
            0, sin(DH(i,3)), cos(DH(i,3));

        T.resize(3,1);
        T << DH(i,2) * cos(DH(i,0)), 
            DH(i,2) * sin(DH(i,0)), 
            DH(i,1);

        JTR.resize(4,4);
        JTR << R, T,
            Eigen::MatrixXd::Zero(1,3), 1;
        
        M[i+1].resize(4,4);
        M[i+1] = M[i]*JTR;

        // update the end-point position of capsule
        pos[i].p1 = M[i+1].block(0,0,3,3) * RoCap[i].p.col(0) + M[i+1].block(0,3,3,1);
        pos[i].p2 = M[i+1].block(0,0,3,3) * RoCap[i].p.col(1) + M[i+1].block(0,3,3,1);
    }
}

void CapPos1(Eigen::MatrixXd base, Eigen::MatrixXd DH, cfslib::math::Capsule RoCap[], Eigen::MatrixXd* M, lineseg* pos){
    int nlink = DH.rows();
    Eigen::MatrixXd R, T, JTR;
    M[0].resize(4,4);
    M[0] << Eigen::MatrixXd::Identity(3,3), base,
            0, 0, 0, 1;

    for (int i=0; i<nlink; ++i){
        R.resize(3,3);
        R << cos(DH(i,0)), -sin(DH(i,0))*cos(DH(i,3)), sin(DH(i,0))*sin(DH(i,3)),
            sin(DH(i,0)), cos(DH(i,0))*cos(DH(i,3)), -cos(DH(i,0))*sin(DH(i,3)),
            0, sin(DH(i,3)), cos(DH(i,3));

        T.resize(3,1);
        T << DH(i,2) * cos(DH(i,0)), 
            DH(i,2) * sin(DH(i,0)), 
            DH(i,1);

        JTR.resize(4,4);
        JTR << R, T,
            Eigen::MatrixXd::Zero(1,3), 1;
        
        M[i+1].resize(4,4);
        M[i+1] = M[i]*JTR;

        // update the end-point position of capsule
        pos[i].p1 = M[i+1].block(0,0,3,3) * RoCap[i].p.col(0) + M[i+1].block(0,3,3,1);
        pos[i].p2 = M[i+1].block(0,0,3,3) * RoCap[i].p.col(1) + M[i+1].block(0,3,3,1);
}

}

double dist_arm_3D_Heu(Eigen::MatrixXd& theta, Eigen::MatrixXd& DH, Eigen::MatrixXd& base, Eigen::MatrixXd& obs, cfslib::math::Capsule cap[]){
    int nstate = theta.rows();
    if(DH.rows() == theta.rows()){
        for (int i=0; i<nstate; ++i){
            DH(i,0) = theta(i,0);
        }
    }
    else{
        for (int i=1; i<1+nstate; ++i){
            DH(i,0) = theta(i-1,0);
        }
    }
    
    
    double d = std::numeric_limits<double>::infinity(); //set d as positive infinity

    // forward kinematics for position calculation under theta
    lineseg pos[DH.rows()];
    Eigen::MatrixXd M[DH.rows()+1];
    CapPos1(base, DH, cap, M, pos);
    
    // calculate the closest distance 
    double dis;
    int link_id;
    for (int i=0; i<DH.rows(); ++i){
        dis = distLinSeg(pos[i].p1, pos[i].p2, obs.block(0,0,3,1), obs.block(0,1,3,1)); // this is 3d scenario. where obstacle position in 3d form
        dis = dis - cap[i].r;
        if (dis < d){
        d = dis;
        link_id = i+1;
        }
    }

    return d;
}

Eigen::MatrixXd central_diff(Eigen::MatrixXd theta, Eigen::MatrixXd DH, Eigen::MatrixXd base, Eigen::MatrixXd obs, cfslib::math::Capsule cap[]){
    double dist = dist_arm_3D_Heu(theta, DH, base, obs, cap);
    Eigen::MatrixXd grad;

    grad.resize(1, theta.rows());
    double eps = 1e-5;

    Eigen::MatrixXd theta_tmp;
    theta_tmp = theta;
    for (int i=0; i<theta.rows(); ++i){
        theta_tmp(i,0) = theta(i,0) + eps/2;
        double dist_h = dist_arm_3D_Heu(theta_tmp, DH, base, obs, cap);
        theta_tmp(i,0) = theta(i,0) - eps/2;
        double dist_l = dist_arm_3D_Heu(theta_tmp, DH, base, obs, cap);
        grad(0,i) = (dist_h - dist_l) / eps;
    }
    return grad;
}

void setInputField(inputfield& line, std::string& data)
{
    /*
    * make sure each txt line has no ' ' in the end
    * make sure there is '\n' at end of txt file 
    */
    // words count 
    int cnt = 0;
    std::string word = ""; 
    for (auto x : data){
        if (x == ' ') 
        {    
            if(cnt == 0){
                // get field
                line.field.assign(word);
                word = ""; // empty word
                cnt++;
            }
            else{
                // get elements
                // cout << word << endl;
                line.elements.push_back(stof(word)); // push back next element in float
                word = ""; // empty word
                cnt++;
            }
        } 
        else
        { 
            word = word + x; 
        }
    }
    // the last element doesn't have blanket behind it
    line.elements.push_back(stof(word)); // push back next element in float
}

int loadPlanningProblemSetting(int& njoint, int& nstate, int& nu, double& margin, int& neq,
            Eigen::MatrixXd& base, std::vector<double>& weight_ref, std::vector<double>& weight_self){
    std::ifstream file("config/cfs_parameters/parameters.txt");
    std::string data;
    inputfield line;
    std::map<std::string, std::vector<float> > dicts;
    while (std::getline(file, data)) {
        // cout << data << "\n";
        // clear line 
        line.field.clear();
        line.elements.clear();
        
        // customized parameter setting
        // get the elements in std::string
        // the first elements is name field
        setInputField(line, data);

        // feed line into dictionary 
        dicts.insert(std::pair<std::string, std::vector<float> >(line.field, line.elements));
    }
    // set the input value
    njoint = (int) dicts["njoint"][0];
    nstate = (int) dicts["nstate"][0];
    nu = (int) dicts["nu"][0];
    margin = (double) dicts["margin"][0];
    neq = (int) dicts["neq"][0];
    base << (double) dicts["base"][0],
            (double) dicts["base"][1],
            (double) dicts["base"][2];
    for (std::vector<float>::const_iterator i = dicts["weight_ref"].begin(); i != dicts["weight_ref"].end(); ++i)
    {
        weight_ref.push_back((double) *i);
    }
    for (std::vector<float>::const_iterator i = dicts["weight_self"].begin(); i != dicts["weight_self"].end(); ++i)
    {
        weight_self.push_back((double) *i);
    }

    // check the input parameter setting value
    std::cout << "njoint is :" << njoint << "\n";
    std::cout << "nstate is :" << nstate << "\n";
    std::cout << "nu is :" << nu << "\n";
    std::cout << "margin is :" << margin << "\n";
    std::cout << "neq is :" << neq << "\n";
    std::cout << "base is :" << base << "\n";
    // set the input value
    std::cout << "all seems good" << "\n";
    // abort();
    return 0;
}

int loadXrefPosition(std::vector<double>& par)
{
    try
    {
        /* code */
        std::fstream myfile;
        par.clear();
        myfile.open ("config/cfs_parameters/xreference_ur.txt");
        if(!myfile.is_open()){
            throw std::runtime_error("xref not found");
        }
        double data;
        while (!myfile.eof()){
            myfile >> data;
            par.push_back(data);
        }
        myfile.close();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        throw;
    }
}

void setReferenceTrajectory(std::vector<double>& ref_vec, Eigen::MatrixXd& ref)
{
    ref.resize(ref_vec.size(),1);
    for (int i=0; i<ref_vec.size(); ++i){
        ref(i,0) = ref_vec[i];
    }
}

}

PlanningCoreOriginal::PlanningCoreOriginal()
{   

}

void PlanningCoreOriginal::Solve(const robot::Robot::ConstPtr& robot_ptr,
                    const query::ProcessedQuery::ConstPtr& processed_query_ptr)
{
    
}

void PlanningCoreOriginal::LoadParams()
{
    // setting parameter, and reference.
    try
    {
        /* code */
        nobs_ = 1; // current set obstacle number to 1
        static_obs_ = false;
        std::vector<double> xref_vec;
        std::cout << "start xref" << "\n";
        helper::loadXrefPosition(xref_vec);
        std::cout << "success xref" << "\n";
        helper::setReferenceTrajectory(xref_vec, xref_);
        base_.resize(3,1);
        helper::loadPlanningProblemSetting(njoint_, nstate_, nu_, margin_, neq_, base_, weight_ref_, weight_self_);
        std::cout << "success planning params" << "\n";
        dim_ = njoint_; // dimension
        // todo read dynamic obstacle from file
        obs_static_.resize(3,2);
        obs_static_ <<  0.45, 0.45,
                        -0.95, -0.95,
                        1.21, 1.21;
        horizon_ = xref_.rows() / dim_; // planning horizon
        // horizon_ = 20;
        nstep_ = horizon_;
        
        // setting xori_
        xori_ = xref_;
        std::cout << "finish the planning problem init" << "\n";
    }
    catch(const std::exception& e)
    {
        // std::cerr << e.what() << '\n';
        throw;
    }
}

void PlanningCoreOriginal::CoreParse(const robot::Robot::ConstPtr& robot_ptr, const query::ProcessedQuery::ConstPtr& processed_query_ptr)
{

    // todo: provide function handle that returns tempopt enable status
    // todo: capture caps

    this->dist_func = [robot_ptr]
        (const std::vector<Eigen::MatrixXd>& args, std::vector<cfslib::math::Capsule>& caps) {

        const Eigen::MatrixXd&  base(robot_ptr->base_frame());
        Eigen::MatrixXd         DH(robot_ptr->DH());

        const Eigen::MatrixXd& theta = args.at(0);
        const Eigen::MatrixXd& obs   = args.at(1);

        int nstate = theta.rows();
        for (int i=0; i<nstate; ++i){
            DH(i,0) = theta(i,0);
        }
        double d = std::numeric_limits<double>::infinity(); //set d as positive infinity

        // forward kinematics for position calculation under theta
        std::vector<helper::lineseg> pos{DH.rows()};
        std::vector<Eigen::MatrixXd> M{DH.rows()+1};
        helper::CapPos(base, DH, caps, M, pos);
        // cout << base << "\n" << DH << "\n";
        
        // calculate the closest distance 
        double dis;
        int link_id;
        for (int i=0; i<nstate; ++i){
            dis = helper::distLinSeg(pos[i].p1, pos[i].p2, obs.block(0,0,3,1), obs.block(0,1,3,1)); // this is 3d scenario. where obstacle position in 3d form
            if (dis < d){
            d = dis;
            link_id = i+1;
            }
        }
        return d;
    };
}

bool PlanningCoreOriginal::CheckInterval(Eigen::MatrixXd x, const Eigen::MatrixXd& dx_limit)
{
    std::cout << "CheckInterval\n";
    int T{int(x.rows()/N_JOINTS)};
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, N_JOINTS, Eigen::RowMajor>> xmat(x.data(), T, N_JOINTS);
    Eigen::MatrixXd dx_max( (xmat.topRows(T-1) - xmat.bottomRows(T-1)).cwiseAbs().colwise().maxCoeff() );
    // std::cout << "x:\n" << xmat << "\n";
    std::cout << "dx_max:\n" << dx_max << "\n";

    if ((dx_limit-dx_max).minCoeff() < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

Status PlanningCoreOriginal::Iteration(const int& iterMax, const double& tolX, const int& resampleMax,
    const Eigen::MatrixXd& dx_limit, robot1::Robot& robot)
{
    try
    {
        std::cout << "init x: \n" << xref_ << "\n";
        xinit_ = xref_;
        int steps{int(xinit_.rows()/N_JOINTS)-1}; // initial steps
        Eigen::MatrixXd xstart(xinit_.topRows(N_JOINTS));
        Eigen::MatrixXd xend(xinit_.bottomRows(N_JOINTS));
        Eigen::MatrixXd xold;

        // record time
        double iter_start_time = 0;
        double iter_end_time = 0;
        double qp_start_time = 0;
        double qp_end_time = 0;
        double process_start_time = 0;
        double process_end_time = 0;

        Status stat = Status::None;
        int resample_iters{-1};

        // The iteration start
        qp_time_.clear();
        cost_.clear();
        iter_start_time = clock();     // record time

        for (int resample_i=0; resample_i<resampleMax; ++resample_i)
        {
            std::cout << "\n==================== ";
            std::cout << "Resample [" << resample_i << "] ";
            std::cout << "====================\n";

            // update trajectoyr properties
            horizon_ = xref_.rows() / dim_; // planning horizon
            nstep_ = horizon_;

            // set Objective from planning problem
            Eigen::MatrixXd Hfull_, f_;
            Eigen::MatrixXd Lfull_, S_; // inequality constraints
            Eigen::MatrixXd Aeq_, beq_; // equality constraints
            Eigen::MatrixXd nLT_, nAeqT_; // negative transpose matrix
            quadprogpp::Matrix<double> G, CE, CI;
            quadprogpp::Vector<double> g0, ce0, ci0, x;

            // initialize reference input
            helper::QPxset(x, xref_);// set initial value of u;

            // set objecive of QP solver 
            auto start_obj = std::chrono::high_resolution_clock::now();
            // pp_->setObjective3d(Hfull_,f_,robot);

            SetCostMatrix3d(Hfull_,f_);

            std::cout << "plnning problem finished the objective setting" << "\n";
            auto stop_obj = std::chrono::high_resolution_clock::now(); 
            auto duration_obj = std::chrono::duration_cast<std::chrono::microseconds>(stop_obj - start_obj); 
            std::cout << "the obj time is: " << double(duration_obj.count())/1000000.0 << "seconds" << "\n"; 
            helper::setObjValue(G, g0, Hfull_, f_);

            iteration_ = -1;
            xold = xref_;
            for (int k=0; k<iterMax; k++)
            {
                // for (int k=0; k<1; k++){
                std::cout << "----------------------" << "\n";
                std::cout << "Iteration " << k << "\n";

                // Processing
                // reset the QP Quadratic term and linear term
                // mandatory: G should be reset after solving one QP problem
                helper::setObjValue(G, g0, Hfull_, f_);
                process_start_time = clock(); 

                // set linear inequality constraint
                LinConstraint(robot, Lfull_, S_);
                nLT_ = -1*Lfull_.transpose(); // negative transpose
                helper::setConstraint(CI, ci0, nLT_, S_); 

                // set linear equality constraint 
                EqConstraint(Aeq_, beq_);
                nAeqT_ = -1*Aeq_.transpose();
                helper::setConstraint(CE, ce0, nAeqT_, beq_); 
                /* uncomment blow to cancel equality constraints*/
                // CE.resize(pp_->horizon_*pp_->njoint_,0);
                // ce0.resize(0);
                // clock stop
                process_end_time = clock();
                process_time_.push_back((process_end_time - process_start_time)/CLOCKS_PER_SEC);

                // Solve the subproblem
                std::cout << "solving QP..." << "\n";
                qp_start_time = clock();

                // if (resample_i > 0)
                // {
                //     std::cout << "G: \n" << G << "\n";
                //     std::cout << "g0: \n" << g0 << "\n";
                //     std::cout << "CE: \n" << CE << "\n";
                //     std::cout << "ce0: \n" << ce0 << "\n";
                //     std::cout << "CI: \n" << CI << "\n";
                //     std::cout << "ci0: \n" << ci0 << "\n";

                //     while (true);
                // }

                tmp_cost = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
                helper::quad2matrix(x, xref_); // update uref_ in planning problem;
                // clock stop
                qp_end_time = clock();
                qp_time_.push_back((qp_end_time - qp_start_time)/CLOCKS_PER_SEC);
                std::cout << "cost temporal cost is: " << tmp_cost << "\n";
                cost_.push_back(tmp_cost);

                // std::cout << "new trajectory is:\n" << pp_->xref_ << "\n";
                //Convergence check
                // compare new xreference
                if (helper::checkConvergence(xref_,xold,tolX)){
                    std::cout << "Converged at step " << k+1 << "\n";
                    iteration_ = k;
                    break;
                }
                // check if inequality constraints are satisifed
                // if (checkLocalOpt(Lfull_, S_, pp_->xref_)){
                //   std::cout << "Local Optima found at step " << k << "\n";
                //   iteration_ = k;
                //   break;
                // }
                // record the current reference trajectory
                xold = xref_;
            }
            
            if (iteration_ < 0)
            {
                stat = Status::CFSNotConverged;
                break;
            }

            if (CheckInterval(xref_, dx_limit))
            {
                resample_iters = resample_i;
                break;
            }
            else
            {
                // resample x ref, increase steps by 10%
                steps = int(ceil(double(steps)*(1.0 + 0.1)));
                std::cout << "Joint step exceeds limit, increasing step to " << steps << "\n";
                Eigen::MatrixXd xref_new(N_JOINTS*(steps+1), 1);
                Eigen::MatrixXd xstep_new((xend-xstart)/double(steps));
                xref_new.topRows(N_JOINTS) = xstart;
                for (int stepi=1; stepi<steps; ++stepi)
                {
                    xref_new.block<N_JOINTS, 1>(stepi*N_JOINTS, 0) = 
                        xref_new.block<N_JOINTS, 1>((stepi-1)*N_JOINTS, 0) + xstep_new;
                }
                xref_new.bottomRows(N_JOINTS) = xend;
                xref_ = xref_new;

                std::cout << "new xref: \n" << xref_ << "\n";
            }
        }

        if (stat == Status::None)
        {
            // cfs converged, check resample status
            if (resample_iters < 0) // dx never feasible
            {
                stat = Status::JointStepExceedsLimit;
            }
            else
            {
                stat = Status::OK;
            }
        }

        soln_ = xref_;
        iter_end_time = clock();
        iteration_time_ = (iter_end_time - iter_start_time)/CLOCKS_PER_SEC;

        return stat;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

int PlanningCoreOriginal::SetCostMatrix3d(Eigen::MatrixXd& H, Eigen::MatrixXd& f)
{
    std::cout << "start cost setting" << "\n";
    Eigen::MatrixXd qd;
    qd.resize(njoint_,njoint_);
    qd <<   1, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 4, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 4;
    std::cout << "finished qd setting" << "\n";
    // assign Q1
    Eigen::MatrixXd Q1 = Eigen::MatrixXd::Zero(horizon_*njoint_,horizon_*njoint_);
    for (int i=0; i < horizon_; i++)
    {
        if(i == horizon_ - 1)
        {
            Q1.block(i*njoint_, i*njoint_, njoint_, njoint_) = qd;
        }
        else
        {
            Q1.block(i*njoint_, i*njoint_, njoint_, njoint_) = qd*0.1;
        }
    }
    std::cout << "finished Q1 setting" << "\n";


    // assign Q2
    // horizon_ = 4;
    Eigen::MatrixXd diag(horizon_*njoint_,horizon_*njoint_);
    Eigen::MatrixXd diag11, diag12, diag21, diag22;
    diag11 = Eigen::MatrixXd::Zero((horizon_-1)*njoint_, njoint_);
    diag12 = Eigen::MatrixXd::Identity((horizon_-1)*njoint_, (horizon_-1)*njoint_);
    diag21 = Eigen::MatrixXd::Zero(njoint_, njoint_);
    diag22 = Eigen::MatrixXd::Zero(njoint_, (horizon_-1)*njoint_);
    diag << diag11, diag12,
            diag21, diag22;
    
    Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(horizon_*njoint_,horizon_*njoint_);
    Eigen::MatrixXd Vdiff(horizon_*njoint_,horizon_*njoint_);
    Vdiff = Id - diag;
    Eigen::MatrixXd Q2 = Vdiff.block(0, 0, (horizon_-1)*njoint_, horizon_*njoint_).transpose() * Vdiff.block(0, 0, (horizon_-1)*njoint_, horizon_*njoint_);
    std::cout << "finished Q2 setting" << "\n";

    // assign Q3
    Eigen::MatrixXd diag2(horizon_*njoint_,horizon_*njoint_);
    Eigen::MatrixXd diag211, diag212, diag221, diag222;
    diag211 = Eigen::MatrixXd::Zero((horizon_-2)*njoint_, njoint_*2);
    diag212 = Eigen::MatrixXd::Identity((horizon_-2)*njoint_, (horizon_-2)*njoint_);
    diag221 = Eigen::MatrixXd::Zero(njoint_*2, njoint_*2);
    diag222 = Eigen::MatrixXd::Zero(njoint_*2, (horizon_-2)*njoint_);
    diag2 << diag211, diag212,
            diag221, diag222;
    Eigen::MatrixXd Adiff = Vdiff - diag + diag2;
    Eigen::MatrixXd Q3 = Adiff.block(0, 0, (horizon_-2)*njoint_, horizon_*njoint_).transpose()  * Adiff.block(0, 0, (horizon_-2)*njoint_, horizon_*njoint_);
    std::cout << "finished Q3 setting" << "\n";

    // set Qref and Qself
    Eigen::MatrixXd Qref = weight_ref_[0] * Q1 + weight_ref_[1] * Q2 +  weight_ref_[2] * Q3;
    Eigen::MatrixXd Qself = weight_self_[0] * Q1 + weight_self_[1] * Q2 +  weight_self_[2] * Q3;
    std::cout << "finished Qref and Qserlf setting" << "\n";

    H = Qref + Qself;
    f = -1 * Qref * xref_;
    std::cout << "finished objective setting" << "\n";

    return 0;
}

int PlanningCoreOriginal::LinConstraint(const robot1::Robot& robot, Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final)
{
    assert(dim_ == njoint_);
    Eigen::MatrixXd Lfull, S;
    Lfull.resize(horizon_, horizon_*dim_); // no need for the start and end 
    S.resize(horizon_, 1); // no need for the start and end
    S = Eigen::MatrixXd::Zero(horizon_, 1);

    // create new variable space 
    Eigen::MatrixXd theta, DH_use;
    cfslib::math::Capsule cap[robot.nlink];
    for (int i=0; i<robot.nlink; ++i){
        cap[i] = robot.cap[i];
    }
    
    // start constraint specification
    Eigen::MatrixXd grad, s, l, d, Diff;
    double distance;
    for (int i=1; i<horizon_-1; ++i)
    {
    // for (int i=0; i<1; ++i){

        theta = xref_.block(i*dim_, 0, dim_, 1);
        DH_use = robot.DH;//.block(0,0,dim_,robot.DH.cols());

        distance = helper::dist_arm_3D_Heu(theta, DH_use, base_, obs_static_, cap);

        d.resize(1,1);
        d(0,0) = distance - margin_;
        
        // numerical gradient
        grad.resize(1, theta.rows());
        grad = helper::central_diff(theta, DH_use, base_, obs_static_, cap);
        Diff.resize(1, horizon_*dim_); // construct gradient of xref in terms of every time step
        Diff = Eigen::MatrixXd::Zero(1, horizon_*dim_); // the rest is zero
        Diff.block(0, i*dim_, 1, dim_) = grad;

        // reset linear constriants 
        s = d - Diff*xref_;
        l = -1*Diff;

        // concatnate s, l to construct S, Lfull
        assert(l.cols() == horizon_*dim_);
        assert(s.cols() == 1 && s.rows() == 1);
        S(i,0) = s(0,0);
        Lfull.block(i, 0, 1, l.cols()) = l;
    }

    // get rid of the first row and end row 
    Lfull_final = Lfull.block(1,0,horizon_-2,Lfull.cols());
    S_final = S.block(1,0,horizon_-2,S.cols());

    return 0;
}

void PlanningCoreOriginal::EqConstraint(Eigen::MatrixXd& Aeq, Eigen::MatrixXd& beq)
{
    Aeq.resize(njoint_*2, njoint_*horizon_);
    beq.resize(njoint_*2,1);

    Aeq << Eigen::MatrixXd::Identity(njoint_,njoint_), Eigen::MatrixXd::Zero(njoint_, (horizon_-1)*njoint_),
            Eigen::MatrixXd::Zero(njoint_, (horizon_-1)*njoint_), Eigen::MatrixXd::Identity(njoint_,njoint_);

    beq << xref_.block(0, 0, njoint_, 1),
            xref_.block((horizon_-1)*njoint_, 0, njoint_, 1);
}

// Temporal Optimization

int PlanningCoreOriginal::SetCostMatrix3dTempopt(Eigen::MatrixXd& H, Eigen::MatrixXd& f)
{
    std::cout << "start cost setting" << "\n";
    H = Eigen::MatrixXd::Identity(horizon_,horizon_);
    f = Eigen::MatrixXd::Zero(horizon_,1);
    std::cout << "finished objective setting" << "\n";
    return 0;
}

int PlanningCoreOriginal::LinConstraintTempopt(Eigen::MatrixXd& Lfull_final, Eigen::MatrixXd& S_final)
{
    Eigen::MatrixXd Lfull, S;
    Eigen::MatrixXd xk0, xk1, xk2;
    double tk0, tk1;
    Eigen::MatrixXd lt0, lt1;
    Eigen::MatrixXd G, gG;
    Eigen::MatrixXd s, l;
    Eigen::MatrixXd Sstack, Lstack;


    // start inequality setting 
    for (int i=0; i<horizon_; ++i){
        if (i >= 1 && i <=horizon_-2){
            xk2 = xref_.block((i+1)*njoint_,0,njoint_,1);
            xk1 = xref_.block(i*njoint_,0,njoint_,1);
            xk0 = xref_.block((i-1)*njoint_,0,njoint_,1);
            tk1 = zref_(i,0); //.block(i,0,1,1);
            tk0 = zref_(i-1,0); //.block(i-1,0,1,1);

            // cout << "x and z is good" << endl;
            // G1 >= 0
            lt0 = -2*(xk2 - xk1) + Eigen::MatrixXd::Ones(njoint_,1)*(amax_*tk1*tk1 + 2*amax_*tk1*tk0);
            lt1 = 2*(xk1 - xk0) + Eigen::MatrixXd::Ones(njoint_,1)*(amax_*tk0*tk0 + 2*amax_*tk1*tk0);
            // cout << "lt is good" << endl;

            G = -2*tk0*(xk2 - xk1) + 2*tk1*(xk1 - xk0) + Eigen::MatrixXd::Ones(njoint_,1)*amax_*tk1*tk0*(tk1+tk0);
            gG.resize(njoint_,horizon_);
            gG << Eigen::MatrixXd::Zero(njoint_,i-1), lt0, lt1, Eigen::MatrixXd::Zero(njoint_,horizon_-i-1);

            // cout << "gG is good" << endl;

            s = -1*(gG * zref_ - G);
            l.resize(njoint_,horizon_);
            l << Eigen::MatrixXd::Zero(njoint_,i-1), -1*lt0, -1*lt1, Eigen::MatrixXd::Zero(njoint_,horizon_-i-1);

            Sstack = cfslib::math::EigenVcat(Sstack, s);
            Lstack = cfslib::math::EigenVcat(Lstack, l);
            // cout << "G1 is good" << endl;

            // G2 >= 0
            lt0 = 2*(xk2 - xk1) + Eigen::MatrixXd::Ones(njoint_,1)*(amax_*tk1*tk1 + 2*amax_*tk1*tk0);
            lt1 = -2*(xk1 - xk0) + Eigen::MatrixXd::Ones(njoint_,1)*(amax_*tk0*tk0 + 2*amax_*tk1*tk0);
            
            G = 2*tk0*(xk2 - xk1) - 2*tk1*(xk1 - xk0) + Eigen::MatrixXd::Ones(njoint_,1)*amax_*tk1*tk0*(tk1+tk0);
            gG.resize(njoint_,horizon_);
            gG << Eigen::MatrixXd::Zero(njoint_,i-1), lt0, lt1, Eigen::MatrixXd::Zero(njoint_,horizon_-i-1);
            
            s = -(gG * zref_ - G);
            l.resize(njoint_,horizon_);
            l << Eigen::MatrixXd::Zero(njoint_,i-1), -1*lt0, -1*lt1, Eigen::MatrixXd::Zero(njoint_,horizon_-i-1);
            Sstack = cfslib::math::EigenVcat(Sstack, s);
            Lstack = cfslib::math::EigenVcat(Lstack, l);
            // cout << "G2 is good " << endl;
            // cout << "middle is good " << endl;
        }

        if (i == 0){
            // v0 - v1 + a1 * t1 = 0
            xk2 = xref_.block((i+1)*njoint_,0,njoint_,1);
            xk1 = xref_.block(i*njoint_,0,njoint_,1);
            l.resize(njoint_,horizon_);
            l << -1*Eigen::MatrixXd::Ones(njoint_,1), Eigen::MatrixXd::Zero(njoint_,horizon_-1);
            s = -((xk1-xk2).cwiseAbs() / amax_).cwiseSqrt();
            Sstack = cfslib::math::EigenVcat(Sstack, s);
            Lstack = cfslib::math::EigenVcat(Lstack, l);
            // cout << "1 is good" << endl;
        }

        if (i == horizon_-1){
            // v0 - v1 + a1 * t1 = 0
            xk1 = xref_.block(i*njoint_,0,njoint_,1);
            xk0 = xref_.block((i-1)*njoint_,0,njoint_,1);
            l.resize(njoint_,horizon_);
            l << Eigen::MatrixXd::Zero(njoint_,horizon_-2), -1*Eigen::MatrixXd::Ones(njoint_,1), Eigen::MatrixXd::Zero(njoint_,1);
            s = -((xk0-xk1).cwiseAbs() / amax_).cwiseSqrt();
            Sstack = cfslib::math::EigenVcat(Sstack, s);
            Lstack = cfslib::math::EigenVcat(Lstack, l);
            // cout << "final is good" << endl;
        }
    }

    Lfull_final = Lstack;
    S_final = Sstack;

    return 0;
}

int PlanningCoreOriginal::IterationTempopt(int iterMax, double tolX, robot1::Robot& robot)
{
    // set Objective from planning problem
        Eigen::MatrixXd Hfull_, f_;
        Eigen::MatrixXd Lfull_, S_; // inequality constraints
        Eigen::MatrixXd Aeq_, beq_; // equality constraints
        Eigen::MatrixXd nLT_, nAeqT_; // negative transpose matrix
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0, x;

        // initilize the time profile reference 
        zref_ = Eigen::MatrixXd::Ones(horizon_,1) * robot.delta_t;
        amax_ = robot.umax;


        // initialize reference input
        helper::QPxset(x, zref_);// set initial value of u;
        xold_tempopt_ = zref_;

        // set objecive of QP solver 
        auto start_obj = std::chrono::high_resolution_clock::now();
        // pp_->setObjective3d(Hfull_,f_,robot);


        SetCostMatrix3dTempopt(Hfull_,f_);

        std::cout << "temporal optimization problem finished the objective setting" << "\n";
        auto stop_obj = std::chrono::high_resolution_clock::now(); 
        auto duration_obj = std::chrono::duration_cast<std::chrono::microseconds>(stop_obj - start_obj); 
        std::cout << "the obj time is: " << double(duration_obj.count())/1000000.0 << "seconds" << "\n"; 
        helper::setObjValue(G, g0, Hfull_, f_);

        // record time
        double iter_start_time = 0;
        double iter_end_time = 0;
        double qp_start_time = 0;
        double qp_end_time = 0;
        double process_start_time = 0;
        double process_end_time = 0;

        // The iteration start
        qp_time_.clear();
        cost_.clear();
        iter_start_time = clock();     // record time

        for (int k=0; k<iterMax; k++){
            // for (int k=0; k<1; k++){
            std::cout << "----------------------" << "\n";
            std::cout << "Iteration " << k << "\n";

            // Processing
            // reset the QP Quadratic term and linear term
            // mandatory: G should be reset after solving one QP problem
            helper::setObjValue(G, g0, Hfull_, f_);
            process_start_time = clock(); 
            std::cout << "finished up the objective" << "\n";

            // set linear inequality constraint
            LinConstraintTempopt(Lfull_, S_);
            nLT_ = -1*Lfull_.transpose(); // negative transpose
            helper::setConstraint(CI, ci0, nLT_, S_); 

            // set linear equality constraint 
            // no equality constraint 
            CE.resize(horizon_,0);
            ce0.resize(0);
            // clock stop
            process_end_time = clock();
            process_time_.push_back((process_end_time - process_start_time)/CLOCKS_PER_SEC);

            // Solve the subproblem
            std::cout << "solving QP ..." << "\n";
            qp_start_time = clock();

            tmp_cost = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
            helper::quad2matrix(x, zref_); // update uref_ in planning problem;
            // clock stop
            qp_end_time = clock();
            qp_time_.push_back((qp_end_time - qp_start_time)/CLOCKS_PER_SEC);
            std::cout << "cost temporal cost is: " << tmp_cost << "\n";
            cost_.push_back(tmp_cost);

            //Convergence check
            // compare new xreference
            if (helper::checkConvergence(zref_,xold_tempopt_,tolX)){
                std::cout << "Converged at step " << k+1 << "\n";
                iteration_ = k;
                break;
            }
            xold_tempopt_ = zref_;
        }
        soln_tempopt_ = zref_;
        iter_end_time = clock();
        iteration_time_ = (iter_end_time - iter_start_time)/CLOCKS_PER_SEC;
        return 0;
}

}

}