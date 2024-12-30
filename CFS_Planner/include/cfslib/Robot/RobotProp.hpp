#pragma once

#include "cfslib/Utils/Common.hpp"
#include "cfslib/Query/ProcessedQuery.hpp"

namespace robot1
{
class Robot {

    public:
        int nlink;
        double umax;
        double margin;
        double pi;
        double delta_t;
        std::string name;
        Eigen::MatrixXd thetamax;
        Eigen::MatrixXd thetadotmax;
        Eigen::MatrixXd l;
        Eigen::MatrixXd DH;
        // Eigen::MatrixXd base;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd Ac;
        Eigen::MatrixXd Bc;
        cfslib::math::Capsule cap[7];
        cfslib::math::lineseg pos[7];
        Eigen::MatrixXd M[8];



    Robot(std::string robot_name)
    {
        int robot_num = parse_name(robot_name);
        switch (robot_num){
            case 1: {
                std::cout << "UR10 go go go" << std::endl;
                UR10property();
                break;
            }
        }
        // initialize the kinematic matrix

        A.resize(12,12);
        A << Eigen::MatrixXd::Identity(6,6), Eigen::MatrixXd::Identity(6,6) * delta_t,
            Eigen::MatrixXd::Zero(6,6),    Eigen::MatrixXd::Identity(6,6);

        B.resize(12,6);
        B << Eigen::MatrixXd::Identity(6,6) * 0.5 * pow(delta_t, 2.0),
            Eigen::MatrixXd::Identity(6,6) * delta_t;

        Ac.resize(6,6);
        Ac << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3) * delta_t,
            Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);

        Bc.resize(6,3);
        Bc << Eigen::MatrixXd::Identity(3,3) * 0.5 * pow(delta_t, 2.0),
            Eigen::MatrixXd::Identity(3,3) * delta_t;
    }


    int parse_name(std::string name)
    {
        std::string gp50, M16iB, ur10;
        ur10 = "UR10";
        if((name.compare(ur10)) == 0)
        {
            std::cout << "robot is identified as " << ur10 << std::endl; 
            std::cout << "------------ start initialize robot property ------------" << std::endl;
            return 1;
        }
        else
        {
            std::cout << "no match robot property found" << std::endl;
            abort();
        } 
        return 0;
    }


    void UR10property()
    {    
        bool with_virtual_base = true;
        name = "UR10";
        pi = M_PI;
        delta_t = 0.5;
        umax = 0.6; // acceleration 

        thetadotmax.resize(6,1);
        thetadotmax << 1,
                       1,
                       1,
                       1,
                       1,
                       1;

        if(with_virtual_base){
            DH.resize(7,4);
            DH << 0,0,0,0,
                0, 0.1273, 0, pi/2,
                0, 0, -0.612, 0, 
                0, 0, -0.5723, 0,
                0, 0.163941, 0, pi/2,
                0, 0.1157, 0, -pi/2,
                0, 0.0922, 0, 0;     
        }
        else{
            DH.resize(6,4);
            DH << 0, 0.1273, 0, pi/2,
                  0, 0, -0.612, 0, 
                  0, 0, -0.5723, 0,
                  0, 0.163941, 0, pi/2,
                  0, 0.1157, 0, -pi/2,
                  0, 0.0922, 0, 0;     
        }
        nlink = DH.rows();
        // std::cout << "robot initialization finished 0" << std::endl;
        // base.resize(3,1);
        // base << 0,  
        //         0,
        //         1.035;

        // std::cout << "robot initialization finished 1" << std::endl;

        // initialize the capsule of robot
        // 0
        if(with_virtual_base){
            cap[0].p.resize(3,2);
            cap[0].p << 0, 0,
                        0, 0,
                        0, 0;      
            cap[0].r = 0;

            cap[1].p.resize(3,2);
            cap[1].p << 0, 0,
                        0.0550, -0.0930,
                        0, 0;      
            cap[1].r = 0.08;
            // 1
            cap[2].p.resize(3,2);
            cap[2].p << 0, 0.612,
                        0, 0,
                        0.17321, 0.17321;    
            cap[2].r = 0.115;
            // 2
            cap[3].p.resize(3,2);
            cap[3].p << 0, 0.5723,
                        0, 0,
                        0.0471, 0.0471;   
            cap[3].r = 0.09;
            // 3
            cap[4].p.resize(3,2);
            cap[4].p << 0, 0,
                        0, -0.116811,
                        0, 0;
            cap[4].r = 0.07;
            // 4
            cap[5].p.resize(3,2);
            cap[5].p << 0, 0,
                        0, 0.1157,
                        0, 0;
            cap[5].r = 0.0675;
            // 5
            cap[6].p.resize(3,2);
            cap[6].p << 0, 0,
                        0, 0,
                        0, 0.14;
            cap[6].r = 0.075;

        }
        else{
            cap[0].p.resize(3,2);
            cap[0].p << 0, 0,
                        0.0550, -0.0930,
                        0, 0;      
            cap[0].r = 0.08;
            // 1
            cap[1].p.resize(3,2);
            cap[1].p << 0, 0.612,
                        0, 0,
                        0.17321, 0.17321;    
            cap[1].r = 0.115;
            // 2
            cap[2].p.resize(3,2);
            cap[2].p << 0, 0.5723,
                        0, 0,
                        0.0471, 0.0471;   
            cap[2].r = 0.09;
            // 3
            cap[3].p.resize(3,2);
            cap[3].p << 0, 0,
                        0, -0.116811,
                        0, 0;
            cap[3].r = 0.07;
            // 4
            cap[4].p.resize(3,2);
            cap[4].p << 0, 0,
                        0, 0.1157,
                        0, 0;
            cap[4].r = 0.0675;
            // 5
            cap[5].p.resize(3,2);
            cap[5].p << 0, 0,
                        0, 0,
                        0, 0.14;
            cap[5].r = 0.075;
        }        

        std::cout << "robot initialization finished" << std::endl;
    }
};
}

