#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "robot_model_gp50.h"

RobotModel_GP50::RobotModel_GP50(void)
{
	/*transformation from Flang to base*/
	X_BF0 << 0.0, 0.0, -1.0, 0.895,
			 0.0, -1.0, 0.0, 0.0,
			 -1.0, 0.0, 0.0, 0.814,
			 0.0, 0.0, 0.0, 1.0;
	/*transformation from Flang to sensor*/
	X_FS <<  1.0, 0.0, 0.0, 0,
			 0.0, 1.0, 0.0, 0.0,
			 0.0, 0.0, 1.0, 0,
			 0.0, 0.0, 0.0, 1.0;
	/*transformation from Flang to tool*/
	X_FT <<  1.0, 0.0, 0.0, 0,
			 0.0, 1.0, 0.0, 0.0,
			 0.0, 0.0, 1.0, 0,
			 0.0, 0.0, 0.0, 1.0;

	X_ST = TransInv(X_FS)* X_FT;

	wlist << 0,0,1,
			0,1,0,
			0,-1,0,
			-1,0,0,
			0,-1,0,
			-1,0,0;

	plist << 0, 0, 0,
			0.155, 0, 0,
			0.155, 0, 0.614,
			0.155, 0, 0.814,
			0.795, 0,0.814,
			0.895, 0,0.814;

	Eigen::Vector3d w1(0,0,1);
	Eigen::Vector3d w2(0,1,0);
	Eigen::Vector3d w3(0,-1,0);
	Eigen::Vector3d w4(-1,0,0);
	Eigen::Vector3d w5(0,-1,0);
	Eigen::Vector3d w6(-1,0,0);

	Eigen::Vector3d p1(0, 0, 0);
	Eigen::Vector3d p2(0.155, 0, 0);
	Eigen::Vector3d p3(0.155, 0, 0.614);
	Eigen::Vector3d p4(0.155, 0, 0.814);
	Eigen::Vector3d p5(0.795, 0,0.814);
	Eigen::Vector3d p6(0.895, 0,0.814);

	for (int i = 0; i < Slist.size(); i++)
	{
		Eigen::VectorXd s(6);
		Eigen::Vector3d w;
		Eigen::Vector3d p;
		// w = wlist.col(i);
		// p << plist(i);
		// s.head(3) << wlist(i);
		// s.tail(3) << -w.cross(p);
		// Slist.col(i) = s;
		// axis.segment(0, 3) = wlist(i);
		// Slist(i).head(3) = wlist(i);
		// cout << Slist.col(i) <<endl;
	}
	Eigen::VectorXd s1(6);
	s1 << w1,-w1.cross(p1);
	Eigen::VectorXd s2(6);
	s2 << w2,-w2.cross(p2);
	Eigen::VectorXd s3(6);
	s3 << w3,-w3.cross(p3);
	Eigen::VectorXd s4(6);
	s4 << w4,-w4.cross(p4);
	Eigen::VectorXd s5(6);
	s5 << w5,-w5.cross(p5);
	Eigen::VectorXd s6(6);
	s6 << w6,-w6.cross(p6);

	Slist.col(0) = s1;
	Slist.col(1) = s2;
	Slist.col(2) = s3;
	Slist.col(3) = s4;
	Slist.col(4) = s5;
	Slist.col(5) = s6;
	// cout <<  Slist <<endl;
	
	cout << "RobotModel_GP50 is being created" << endl;
}

RobotModel_GP50::~RobotModel_GP50()
{
	cout << "RobotModel_GP50 is being deleted" << endl;
}


Eigen::MatrixXd RobotModel_GP50::calFK(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd X_BF = mr::FKinSpace(X_BF0, Slist, thetaList);

	if(flag == 'F')
	{
		return X_BF;
	}
	else if(flag == 'S')
	{
		return X_BF * X_FS;
	}
	else if(flag == 'T')
	{
		return X_BF * X_FT;
	}

	return X_BF;
}

Eigen::VectorXd RobotModel_GP50::calIK(const Eigen::VectorXd& thetalistguess,const Eigen::MatrixXd& T)
{
	Eigen::VectorXd	thetalist(6);
	thetalist = thetalistguess;
	double eomg = 0.01;
	double ev = 0.001;
	bool IK_success = false;
	IK_success = mr::IKinSpace(Slist, X_BF0, T, thetalist, eomg, ev);
	if(IK_success == false)
	{
		printf("IKinSpace false");
		thetalist << 0,0,0,0,0,0;
		return thetalist;
	}
	else
	{
		return thetalist;
	}

}

Eigen::MatrixXd RobotModel_GP50::calBodyJaco(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd Js_BF,Jb_BF;
	Eigen::MatrixXd X_BF;
	X_BF = calFK(thetaList,flag);
	Js_BF = JacobianSpace(Slist, thetaList);

	/*convert spatial Jaco to body Jaco*/
	Jb_BF = Adjoint(TransInv(X_BF)) * Js_BF;

	if(flag == 'F')
	{
		return Jb_BF;
	}
	else if(flag == 'S')
	{

		// return X_BF * X_FS;
	}
	else if(flag == 'T')
	{

		// return X_BF * X_FT;
	}
	return Jb_BF;
}

Eigen::MatrixXd RobotModel_GP50::calGeomJaco(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd Jb_BF;
	Eigen::MatrixXd Jg_BF;
	Jb_BF = calBodyJaco(thetaList,flag);
	Eigen::MatrixXd X_BF,R_BF;
	X_BF = calFK(thetaList,flag);

	// Get top left 3x3 corner
	R_BF = X_BF.block<3, 3>(0, 0);
	Eigen::Matrix<double, 6, 6> R_BF66 = Eigen::Matrix<double, 6, 6>::Zero();
	R_BF66.topLeftCorner<3, 3>() = R_BF;
	R_BF66.bottomRightCorner<3, 3>() = R_BF;
	return R_BF66 * Jb_BF;
}